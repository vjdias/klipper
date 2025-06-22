# Controle de movimento em malha fechada via FPGA conectado ao MCU.
# Este módulo permite configurar parâmetros estáticos do controlador FPGA
# e enviar comandos PWM em tempo real utilizando a fila de comandos do MCU.
#
# Copyright (C) 2025  Your Name <you@domain.com>
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import bus


class FPGALoopController:
    """Controlador de malha fechada baseado em FPGA."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        # Opções para uso de SPI via bitbang (software) quando a MCU não
        # possui um barramento SPI livre.  Caso qualquer um dos pinos seja
        # especificado, o helper abaixo automaticamente os utiliza.
        self.sw_mosi_pin = config.get('spi_software_mosi_pin', None)
        self.sw_miso_pin = config.get('spi_software_miso_pin', None)
        self.sw_sclk_pin = config.get('spi_software_sclk_pin', None)

        # Configura objeto SPI utilizando o helper padronizado. Este helper
        # entende as opções "spi_bus" e "spi_software_*_pin" e gera os
        # comandos apropriados para o MCU.
        self.spi = bus.MCU_SPI_from_config(
            config, 0, pin_option='spi_cs_pin', default_speed=1000000)

        # Definição de eixos associados a steppers
        self._axis_defs = {}
        for axis in ['x', 'y', 'z', 'a', 'b']:
            sec_name = config.get(f'axis_{axis}', None)
            if sec_name is None:
                continue
            scfg = config.getsection(sec_name)
            microsteps = scfg.getint('microsteps', note_valid=False)
            self._axis_defs[axis] = (sec_name, microsteps)
        if not self._axis_defs:
            raise self.printer.config_error(
                "[fpga_loop] requer pelo menos um eixo configurado")

        # Ganhos PID configurados para o controlador
        self.pid_P = config.getfloat('pid_P', minval=0.0)
        self.pid_I = config.getfloat('pid_I', minval=0.0)
        self.pid_D = config.getfloat('pid_D', minval=0.0)

        self.update_interval = config.getfloat('update_interval', 0.005,
                                               above=0.)
        self.invert_enable = config.getboolean('invert_enable', False)

        # MCU já está disponível nesta fase, pois é criado antes das seções
        # extras. Registramos o callback de configuração imediatamente para que
        # os comandos sejam incluídos na configuração inicial enviada ao MCU.
        self._mcu = self.printer.lookup_object('mcu')
        self.oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)

        # Variáveis usadas em tempo de execução
        self._axes = {}
        self.cmd_queue = None
        self._queue_pwm = None
        self._set_fpga_cmd = None
        self._last_clock = 0

        # Rotina de inicialização após conexão com o MCU
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)

    def _handle_connect(self):
        # Todas as impressoras já estão instanciadas neste ponto; podemos
        # localizar os steppers para cada eixo e substituir o gerador de passos
        force_move = self.printer.lookup_object('force_move')
        toolhead = self.printer.lookup_object('toolhead')
        for axis, (sname, microsteps) in self._axis_defs.items():
            stepper = force_move.lookup_stepper(sname)
            self._axes[axis] = (stepper, microsteps)
            # Desativa o gerador de passos normal (queue_step) para este eixo.
            # Os pulsos serão emitidos exclusivamente pelo FPGA.
            toolhead.unregister_step_generator(stepper.generate_steps)
            if self._set_fpga_cmd is None:
                self._set_fpga_cmd = self._mcu.lookup_command(
                    "stepper_set_fpga oid=%c enable=%c")
            self._set_fpga_cmd.send([stepper.get_oid(), 1])
        # Passa a utilizar o gerador de passos deste módulo, que repassa
        # comandos ao FPGA em vez de programar pulsos no MCU.
        toolhead.register_step_generator(self._stepgen_fpga)
        self._init_pwm()

    def _build_config(self):
        # Durante a montagem da configuração podemos finalmente resolver os
        # steppers correspondentes, caso ainda não o tenhamos feito.
        if not self._axes:
            force_move = self.printer.lookup_object('force_move')
            for axis, (sname, microsteps) in self._axis_defs.items():
                stepper = force_move.lookup_stepper(sname)
                self._axes[axis] = (stepper, microsteps)

        parts = [
            f"config_fpga oid={self.oid}",
            f"spi_oid={self.spi.get_oid()}"
        ]
        # O firmware espera parametros para todos os cinco eixos. Caso algum
        # nao tenha sido definido no arquivo de configuracao, utiliza-se o
        # valor zero para indicar que ele esta desativado.
        for axis in ['x', 'y', 'z', 'a', 'b']:
            stepper, microsteps = self._axes.get(axis, (None, 0))
            oid = stepper.get_oid() if stepper else 0
            parts.append(f"axis_{axis}_oid={oid}")
            parts.append(f"microsteps_{axis}={microsteps}")
        # Os ganhos PID são configurados como inteiros 16 bits no MCU.
        pid_scale = int(self._mcu.get_constant_float('FPGA_PWM_MAX'))
        p_gain = int(self.pid_P * pid_scale + 0.5)
        i_gain = int(self.pid_I * pid_scale + 0.5)
        d_gain = int(self.pid_D * pid_scale + 0.5)
        parts.append(f"pid_P={p_gain}")
        parts.append(f"pid_I={i_gain}")
        parts.append(f"pid_D={d_gain}")
        cmd = ' '.join(parts)
        self._mcu.add_config_cmd(cmd)
        if self.sw_mosi_pin or self.sw_miso_pin or self.sw_sclk_pin:
            logging.info(
                "FPGA Loop '%s' usando SPI por software: MOSI=%s MISO=%s SCLK=%s",
                self.name, self.sw_mosi_pin, self.sw_miso_pin, self.sw_sclk_pin)
        logging.info("FPGA Loop '%s' configurado: %s", self.name, cmd)

    def _init_pwm(self):
        self.cmd_queue = self._mcu.alloc_command_queue()
        # O firmware define o valor maximo de PWM atraves da constante
        # FPGA_PWM_MAX. Obtemos esse valor para converter o duty (0.0-1.0)
        self._pwm_max = self._mcu.get_constant_float('FPGA_PWM_MAX')
        self._queue_pwm = self._mcu.lookup_command(
            "queue_fpga_pwm oid=%c clock=%u duty=%hu",
            cq=self.cmd_queue)
        curtime = self.reactor.monotonic()
        curclock = self._mcu.print_time_to_clock(
            self._mcu.estimated_print_time(curtime))
        self._last_clock = curclock + self._mcu.print_time_to_clock(0.200)
        logging.info("FPGA Loop '%s': fila PWM inicializada", self.name)

    def _stepgen_fpga(self, flush_time):
        """Gerador de passos substituto que envia atualizacoes de PWM."""
        try:
            self.set_pwm(flush_time, 0.)
        except self.printer.command_error as e:
            logging.error("FPGA stepgen erro: %s", str(e))
        return

    def set_pwm(self, print_time, duty):
        if self._queue_pwm is None:
            raise self.printer.command_error("FPGA loop não inicializado")
        clock = self._mcu.print_time_to_clock(print_time)
        clock = max(self._last_clock, clock)
        duty = max(0.0, min(1.0, duty))
        ivalue = int(duty * self._pwm_max + 0.5)
        self._queue_pwm.send([self.oid, clock, ivalue],
                             minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock
        logging.debug("FPGA Loop '%s': duty=%.3f enviado em clock=%d",
                      self.name, duty, clock)


def load_config_prefix(config):
    return FPGALoopController(config)
