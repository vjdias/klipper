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

// Valor maximo que representa duty cycle em queue_fpga_move
#define FPGA_PWM_MAX 65535
DECL_CONSTANT("FPGA_PWM_MAX", FPGA_PWM_MAX);

// Tamanho maximo do buffer interno de movimentos
#define FPGA_BUFFER_SIZE 64
DECL_CONSTANT("FPGA_BUFFER_SIZE", FPGA_BUFFER_SIZE);

// Estrutura para cada movimento enfileirado ao FPGA
struct fpga_move {
    struct move_node node;
    uint32_t waketime; // instante para envio
    uint32_t interval; // intervalo base entre passos
    uint16_t count;    // quantidade de passos
    int16_t add;       // incremento por passo
};

// Estrutura principal associada a um controlador FPGA
struct fpga_controller {
    struct timer timer;            // gerencia execucao dos comandos enfileirados
    struct spidev_s *spi;         // interface SPI utilizada
    struct move_queue_head mq;    // fila de movimentos
    uint8_t queued;               // numero de movimentos aguardando
};

/********************************************************************
 * Rotina de envio ao FPGA
 ********************************************************************/

// Envia o proximo movimento ao FPGA quando chega o tempo indicado
static uint_fast8_t
fpga_move_event(struct timer *t)
{
    struct fpga_controller *fc = container_of(t, struct fpga_controller, timer);
    struct move_node *mn = move_queue_pop(&fc->mq);
    struct fpga_move *m = container_of(mn, struct fpga_move, node);

    // Mensagem: cmd=1, intervalo, count, add (little-endian)
    uint8_t msg[9] = { 1,
        m->interval, m->interval>>8, m->interval>>16, m->interval>>24,
        m->count & 0xff, m->count>>8,
        m->add & 0xff, m->add >> 8 };
    spidev_transfer(fc->spi, 0, sizeof(msg), msg);
    move_free(m);
    if (fc->queued)
        fc->queued--;

    if (move_queue_empty(&fc->mq))
        return SF_DONE;

    struct fpga_move *next = container_of(move_queue_first(&fc->mq),
                                          struct fpga_move, node);
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
    fc->timer.func = fpga_move_event;
    fc->spi = spidev_oid_lookup(args[1]);
    move_queue_setup(&fc->mq, sizeof(struct fpga_move));
    fc->queued = 0;

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

// Enfileira novo movimento para o FPGA
void
command_queue_fpga_move(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    if (fc->queued >= FPGA_BUFFER_SIZE)
        shutdown("FPGA buffer full");
    struct fpga_move *m = move_alloc();
    m->waketime = args[1];
    m->interval = args[2];
    m->count = args[3];
    m->add = args[4];

    irq_disable();
    int need_add = move_queue_push(&m->node, &fc->mq);
    fc->queued++;
    irq_enable();
    if (!need_add)
        return;

    sched_del_timer(&fc->timer);
    fc->timer.waketime = m->waketime;
    sched_add_timer(&fc->timer);
}
DECL_COMMAND(command_queue_fpga_move,
             "queue_fpga_move oid=%c clock=%u interval=%u count=%hu add=%hi");

// Retorna quantidade de espacos livres no buffer do FPGA
void
command_query_fpga_buffer(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    uint8_t free = FPGA_BUFFER_SIZE - fc->queued;
    sendf("fpga_buffer oid=%c free=%c", args[0], free);
}
DECL_COMMAND(command_query_fpga_buffer, "query_fpga_buffer oid=%c");

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

