
# Controle de movimento em malha fechada via FPGA conectado ao MCU.
# Este módulo permite configurar parâmetros estáticos do controlador FPGA
# e enviar comandos de movimento em tempo real utilizando a fila do MCU.
#
# Copyright (C) 2025  Your Name <you@domain.com>
# Este arquivo pode ser distribuído sob os termos da licença GNU GPLv3.

import logging
from . import bus


class FPGALoopController:
    """Controlador de malha fechada implementado em FPGA."""

    def __init__(self, config):
        # Objetos principais do Klipper
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        # Opções para uso de SPI via bitbang caso o MCU não possua barramento
        # dedicado. Se algum dos pinos for especificado, o helper tratará o
        # acesso em software automaticamente.
        self.sw_mosi_pin = config.get('spi_software_mosi_pin', None)
        self.sw_miso_pin = config.get('spi_software_miso_pin', None)
        self.sw_sclk_pin = config.get('spi_software_sclk_pin', None)

        # Cria objeto SPI utilizando helper padronizado do Klipper
        # Velocidade de comunicação pode ser ajustada via 'spi_speed'
        # (padrão 1MHz se não especificado)
        spi_speed = config.getint('spi_speed', 1000000, minval=100000)
        self.spi_speed = spi_speed
        self.spi = bus.MCU_SPI_from_config(
            config, 0, pin_option='spi_cs_pin', default_speed=spi_speed)

        # Pino usado como "trigger" indicando que o FPGA terminou um movimento
        ppins = self.printer.lookup_object('pins')
        tpin_params = ppins.lookup_pin(config.get('trigger_pin'))
        if tpin_params['chip'] != self.spi.get_mcu():
            raise config.error('trigger_pin deve estar no mesmo MCU do SPI')
        self.trigger_pin = tpin_params['pin']

        # Informações dos eixos controlados pelo FPGA
        self._axis_defs = {}
        self.pid_gains = {}
        # Seções do tipo [fpga_stepper <axis>]
        for scfg in config.get_prefix_sections('fpga_stepper '):
            axis = scfg.get_name().split()[1].lower()
            micro = scfg.getint('microsteps', note_valid=False)
            self._axis_defs[axis] = (scfg.get_name(), micro)
            p = scfg.getfloat('pid_P', 0.0)
            i = scfg.getfloat('pid_I', 0.0)
            d = scfg.getfloat('pid_D', 0.0)
            self.pid_gains[axis] = (p, i, d)

        if not self._axis_defs:
            raise self.printer.config_error(
                "É necessário definir pelo menos um [fpga_stepper]")

        self.update_interval = config.getfloat('update_interval', 0.005,
                                               above=0.)
        self.invert_enable = config.getboolean('invert_enable', False)

        # Variáveis para uso após a conexão
        self._mcu = self.printer.lookup_object('mcu')
        self.oid = self._mcu.create_oid()
        # registra comando de configuração imediatamente para que
        # o MCU crie o controlador antes do restante da conexão
        self._mcu.register_config_callback(self._build_config)

        self._axes = {}
        self.cmd_queue = None
        self._queue_move = None
        self._queue_pmove = None
        self._query_buffer = None
        self._query_pos_cmd = None
        self._set_fpga_cmd = None
        self._buffer_free = 0
        self.buffer_size = 0
        self._pwm_max = 0
        self._last_clock = 0

        # Processa conexão quando MCU estiver pronta
        # Quando o MCU concluir a fase de conexão, configurará os
        # steppers associados e iniciará a comunicação com o FPGA.
        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        self.printer.register_event_handler(
            'klippy:disconnect', self._handle_disconnect)

    def _handle_connect(self):
        """Configura steppers e inicializa comunicação."""
        force_move = self.printer.lookup_object('force_move')
        toolhead = self.printer.lookup_object('toolhead')
        for axis, (sec_name, micro) in self._axis_defs.items():
            stepper = force_move.lookup_stepper(sec_name)
            self._axes[axis] = (stepper, micro)
            # Desativa gerador de passos tradicional deste eixo
            toolhead.unregister_step_generator(stepper.generate_steps)
            if self._set_fpga_cmd is None:
                self._set_fpga_cmd = self._mcu.lookup_command(
                    "stepper_set_fpga oid=%c enable=%c")
            self._set_fpga_cmd.send([stepper.get_oid(), 1])
        toolhead.register_step_generator(self._stepgen_fpga)
        self._mcu.register_config_callback(self._build_config)
        self._init_comm()

    def _handle_disconnect(self):
        """Restaura geradores de passo ao desconectar."""
        toolhead = self.printer.lookup_object('toolhead')
        for stepper, _ in self._axes.values():
            toolhead.register_step_generator(stepper.generate_steps)
            if self._set_fpga_cmd is not None:
                self._set_fpga_cmd.send([stepper.get_oid(), 0])
        try:
            toolhead.unregister_step_generator(self._stepgen_fpga)
        except ValueError:
            pass

    def _build_config(self):
        """Envia configuração estática ao MCU/FPGA."""
        parts = [
            f"config_fpga oid={self.oid}",
            f"spi_oid={self.spi.get_oid()}",
            f"trigger_pin={self.trigger_pin}"
        ]
        if not self._pwm_max:
            try:
                self._pwm_max = self._mcu.get_constant_float('FPGA_PWM_MAX')
            except Exception:
                self._pwm_max = 65535.
        for axis in ['x', 'y', 'z', 'a', 'b']:
            stepper, micro = self._axes.get(axis, (None, 0))
            oid = stepper.get_oid() if stepper else 0
            p, i, d = self.pid_gains.get(axis, (0.0, 0.0, 0.0))
            parts.append(f"axis_{axis}_oid={oid}")
            parts.append(f"microsteps_{axis}={micro}")
            parts.append(f"pid_P_{axis}={int(p*self._pwm_max+0.5)}")
            parts.append(f"pid_I_{axis}={int(i*self._pwm_max+0.5)}")
            parts.append(f"pid_D_{axis}={int(d*self._pwm_max+0.5)}")
        cmd = ' '.join(parts)
        self._mcu.add_config_cmd(cmd)
        logging.info("FPGA Loop '%s' configurado: %s", self.name, cmd)

    def _init_comm(self):
        """Inicializa fila de comandos e estado do buffer."""
        self.cmd_queue = self._mcu.alloc_command_queue()
        self._pwm_max = self._mcu.get_constant_float('FPGA_PWM_MAX')
        self.buffer_size = int(self._mcu.get_constants().get(
            'FPGA_BUFFER_SIZE', 0))
        self._queue_move = self._mcu.lookup_command(
            "queue_fpga_move oid=%c clock=%u interval=%u count=%hu add=%hi",
            cq=self.cmd_queue)
        self._queue_pmove = self._mcu.lookup_command(
            "queue_fpga_pmove oid=%c clock=%u"
            " interval_x=%u count_x=%hu add_x=%hi"
            " interval_y=%u count_y=%hu add_y=%hi"
            " interval_z=%u count_z=%hu add_z=%hi"
            " interval_a=%u count_a=%hu add_a=%hi"
            " interval_b=%u count_b=%hu add_b=%hi",
            cq=self.cmd_queue)
        self._query_buffer = self._mcu.lookup_query_command(
            "query_fpga_buffer oid=%c",
            "fpga_buffer oid=%c free=%c")
        self._query_pos_cmd = self._mcu.lookup_query_command(
            "fpga_stepper_get_position oid=%c",
            ("fpga_stepper_position oid=%c pos_x=%i pos_y=%i "
             "pos_z=%i pos_a=%i pos_b=%i"))
        # Mensagens assíncronas informando execuções concluídas
        self._mcu.register_response(self._handle_exec, "fpga_exec", self.oid)
        # Assume buffer vazio no início; o primeiro movimento ajustará o valor
        self._buffer_free = self.buffer_size
        curtime = self.reactor.monotonic()
        curclock = self._mcu.print_time_to_clock(
            self._mcu.estimated_print_time(curtime))
        self._last_clock = curclock + self._mcu.print_time_to_clock(0.200)
        logging.info("FPGA Loop '%s': comunicação iniciada", self.name)

    def _stepgen_fpga(self, flush_time):
        """Gera comandos para todos os eixos controlados pelo FPGA."""
        try:
            # Se existir mais de um eixo associado, envie movimento paralelo
            if len(self._axes) > 1:
                intervals = [0] * 5
                counts = [0] * 5
                adds = [0] * 5
                self.set_parallel_move(intervals, counts, adds)
            else:
                # Mantém comportamento anterior para único eixo
                self.set_move(0, 0, 0.)
        except self.printer.command_error as e:
            logging.error("FPGA stepgen erro: %s", str(e))
        return

    def _handle_exec(self, params):
        """Recebe sinal de conclusão de movimento do FPGA."""
        executed = params.get('executed', 1)
        self._buffer_free = min(self.buffer_size,
                               self._buffer_free + executed)

    def set_move(self, interval, count, at_time=None, add=0):
        """Enfileira movimento para o FPGA."""
        if self._queue_move is None:
            raise self.printer.command_error("FPGA loop não inicializado")
        if at_time is None:
            at_time = self._mcu.clock_to_print_time(self._last_clock)
            at_time += self.update_interval
        clock = self._mcu.print_time_to_clock(at_time)
        min_clock = self._last_clock
        if clock < min_clock + self._mcu.print_time_to_clock(0.010):
            clock = min_clock + self._mcu.print_time_to_clock(0.010)
        # Aguarda espaço no buffer do FPGA
        while self._buffer_free == 0:
            resp = self._query_buffer.send([self.oid])
            self._buffer_free = resp.get('free', 0)
            if self._buffer_free == 0:
                self.reactor.pause(0.001)
        self._queue_move.send([self.oid, clock, interval, count, add],
                              minclock=min_clock, reqclock=clock)
        self._buffer_free -= 1
        self._last_clock = clock

    def set_parallel_move(self, intervals, counts, adds, at_time=None):
        """Enfileira movimento paralelo de todos os eixos."""
        if self._queue_pmove is None:
            raise self.printer.command_error("FPGA loop não inicializado")
        if at_time is None:
            at_time = self._mcu.clock_to_print_time(self._last_clock)
            at_time += self.update_interval
        clock = self._mcu.print_time_to_clock(at_time)
        min_clock = self._last_clock
        if clock < min_clock + self._mcu.print_time_to_clock(0.010):
            clock = min_clock + self._mcu.print_time_to_clock(0.010)
        while self._buffer_free == 0:
            resp = self._query_buffer.send([self.oid])
            self._buffer_free = resp.get('free', 0)
            if self._buffer_free == 0:
                self.reactor.pause(0.001)
        params = [self.oid, clock]
        for i in range(5):
            params += [intervals[i], counts[i], adds[i]]
        self._queue_pmove.send(params, minclock=min_clock, reqclock=clock)
        self._buffer_free -= 1
        self._last_clock = clock

    def query_positions(self):
        """Retorna posições atuais fornecidas pelo FPGA."""
        if self._query_pos_cmd is None:
            raise self.printer.command_error("FPGA loop não inicializado")
        resp = self._query_pos_cmd.send([self.oid])
        return {a: resp.get('pos_' + a, 0) for a in 'xyzab'}



def load_config_prefix(config):
    """Inicializa modulo a partir do arquivo de configuracao."""
    return FPGALoopController(config)
