// Commands for FPGA based closed-loop motor controller
//
// Copyright (C) 2025  Your Name
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "board/irq.h"           // irq_disable
#include "board/gpio.h"          // gpio_out_write
#include "board/misc.h"          // timer_is_before
#include "basecmd.h"             // oid_alloc
#include "command.h"             // DECL_COMMAND
#include "sched.h"               // sched_add_timer
#include "spicmds.h"            // spidev_transfer
#include "stepper.h"             // stepper_set_fpga

// Valor maximo que representa duty cycle em queue_fpga_pwm
#define FPGA_PWM_MAX 65535
DECL_CONSTANT("FPGA_PWM_MAX", FPGA_PWM_MAX);

// Estrutura para cada atualizacao de duty enviado ao FPGA
struct fpga_pwm_move {
    struct move_node node;
    uint32_t waketime; // momento para envio no clock MCU
    uint16_t duty;     // valor de duty (0..FPGA_PWM_MAX)
};

// Estrutura principal associada a um controlador FPGA
struct fpga_controller {
    struct timer timer;            // gerencia execucao dos comandos enfileirados
    struct spidev_s *spi;         // interface SPI utilizada
    struct move_queue_head mq;    // fila de atualizacoes PWM
};

/********************************************************************
 * Rotina de envio ao FPGA
 ********************************************************************/

// Envia o proximo comando PWM ao FPGA quando chega o tempo indicado
static uint_fast8_t
fpga_pwm_event(struct timer *t)
{
    struct fpga_controller *fc = container_of(t, struct fpga_controller, timer);
    struct move_node *mn = move_queue_pop(&fc->mq);
    struct fpga_pwm_move *m = container_of(mn, struct fpga_pwm_move, node);

    // Monta mensagem simples: cmd=1, duty em formato little-endian
    uint8_t msg[3] = { 1, m->duty & 0xff, m->duty >> 8 };
    spidev_transfer(fc->spi, 0, sizeof(msg), msg);
    move_free(m);

    if (move_queue_empty(&fc->mq))
        return SF_DONE;

    struct fpga_pwm_move *next = container_of(move_queue_first(&fc->mq),
                                              struct fpga_pwm_move, node);
    t->waketime = next->waketime;
    return SF_RESCHEDULE;
}

/********************************************************************
 * Comandos acessíveis pelo host
 ********************************************************************/

// Configura um novo controlador FPGA e envia parametros basicos por SPI
void
command_config_fpga(uint32_t *args)
{
    struct fpga_controller *fc = oid_alloc(args[0], command_config_fpga,
                                           sizeof(*fc));
    fc->timer.func = fpga_pwm_event;
    fc->spi = spidev_oid_lookup(args[1]);
    move_queue_setup(&fc->mq, sizeof(struct fpga_pwm_move));

    // Transmite configuracao estatica ao FPGA. O formato exato da
    // mensagem eh definido pela implementacao do hardware e pode
    // incluir, por exemplo, o oid dos steppers e parametros PID.
    uint8_t cfg[2] = { 0 }; // cmd=0 indica configuracao
    spidev_transfer(fc->spi, 0, sizeof(cfg), cfg);
}
DECL_COMMAND(command_config_fpga,
             "config_fpga oid=%c spi_oid=%c"
             " axis_x_oid=%c microsteps_x=%hu"
             " axis_y_oid=%c microsteps_y=%hu"
             " axis_z_oid=%c microsteps_z=%hu"
             " axis_a_oid=%c microsteps_a=%hu"
             " axis_b_oid=%c microsteps_b=%hu"
             " pid_P=%hu pid_I=%hu pid_D=%hu");

// Enfileira nova atualizacao de duty para o FPGA
void
command_queue_fpga_pwm(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    struct fpga_pwm_move *m = move_alloc();
    m->waketime = args[1];
    m->duty = args[2];

    irq_disable();
    int need_add = move_queue_push(&m->node, &fc->mq);
    irq_enable();
    if (!need_add)
        return;

    sched_del_timer(&fc->timer);
    fc->timer.waketime = m->waketime;
    sched_add_timer(&fc->timer);
}
DECL_COMMAND(command_queue_fpga_pwm,
             "queue_fpga_pwm oid=%c clock=%u duty=%hu");

// Limpa filas e pinos durante desligamento
void
fpga_shutdown(void)
{
    uint8_t i; struct fpga_controller *fc;
    foreach_oid(i, fc, command_config_fpga) {
        move_queue_clear(&fc->mq);
    }
}
DECL_SHUTDOWN(fpga_shutdown);

