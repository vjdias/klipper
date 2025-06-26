// Comandos para controlador de motores em malha fechada utilizando FPGA
//
// Copyright (C) 2025  Your Name
// Este arquivo pode ser distribuído sob os termos da licença GNU GPLv3.

#include <string.h>
#include "board/irq.h"           // irq_disable
#include "board/gpio.h"          // gpio_out_write
#include "board/misc.h"          // timer_is_before
#include "basecmd.h"             // oid_alloc
#include "command.h"             // DECL_COMMAND
#include "sched.h"               // sched_add_timer
#include "spicmds.h"            // spidev_transfer
#include "stepper.h"             // stepper_set_fpga

// Valor máximo representando duty cycle ou ganho em comunicações
#define FPGA_PWM_MAX 65535
DECL_CONSTANT("FPGA_PWM_MAX", FPGA_PWM_MAX);

// Capacidade máxima do buffer de movimentos
#define FPGA_BUFFER_SIZE 64
DECL_CONSTANT("FPGA_BUFFER_SIZE", FPGA_BUFFER_SIZE);

// Estrutura para cada movimento simples enfileirado ao FPGA
struct fpga_move {
    struct move_node node;
    uint8_t type;        // 0=simples, 1=paralelo
    uint32_t waketime;   // instante para envio
    uint32_t interval;   // intervalo base entre passos
    uint16_t count;      // quantidade de passos
    int16_t  add;        // incremento por passo
};

// Estrutura contendo dados para mover vários eixos em paralelo
struct fpga_pmove {
    struct move_node node;
    uint8_t type;                // 0=simples, 1=paralelo
    uint32_t waketime;           // instante para envio
    uint32_t interval[5];        // intervalo de cada eixo
    uint16_t count[5];           // quantidade de passos por eixo
    int16_t  add[5];             // incremento por passo de cada eixo
};

// Estrutura principal de cada controlador FPGA
struct fpga_controller {
    uint8_t oid;                 // identificador do objeto
    struct timer timer;          // gerenciamento de eventos
    struct spidev_s *spi;        // interface SPI usada
    struct gpio_in trigger_pin;  // pino indicando que o FPGA finalizou
    uint32_t poll_ticks;         // intervalo de verificação em clocks
    struct move_queue_head mq;   // fila de movimentos
    uint8_t queued;              // quantidade de movimentos na fila
    uint16_t pid_P[5];           // ganhos PID por eixo
    uint16_t pid_I[5];
    uint16_t pid_D[5];
};

/********************************************************************
 * Envio de movimentos ao FPGA
 ********************************************************************/

// Quando chega o momento indicado, transmite o próximo movimento via SPI
static uint_fast8_t
fpga_move_event(struct timer *t)
{
    struct fpga_controller *fc = container_of(t, struct fpga_controller, timer);

    if (move_queue_empty(&fc->mq))
        return SF_DONE;

    struct fpga_move *m = container_of(move_queue_first(&fc->mq),
                                       struct fpga_move, node);
    uint32_t cur = timer_read_time();
    if (!gpio_in_read(fc->trigger_pin) || timer_is_before(cur, m->waketime)) {
        t->waketime = cur + fc->poll_ticks;
        return SF_RESCHEDULE;
    }

    move_queue_pop(&fc->mq);
    if (m->type == 0) {
        uint8_t msg[9] = { 1,
            m->interval, m->interval>>8, m->interval>>16, m->interval>>24,
            m->count & 0xff, m->count>>8,
            m->add & 0xff, m->add >> 8 };
        spidev_transfer(fc->spi, 0, sizeof(msg), msg);
    } else {
        struct fpga_pmove *pm = (struct fpga_pmove *)m;
        uint8_t msg[1 + 5*8];
        msg[0] = 2; // comando para movimento paralelo
        uint8_t *p = &msg[1];
        for (int i=0; i<5; i++) {
            uint32_t iv = pm->interval[i];
            *p++ = iv; *p++ = iv>>8; *p++ = iv>>16; *p++ = iv>>24;
            uint16_t c = pm->count[i];
            *p++ = c; *p++ = c>>8;
            int16_t a = pm->add[i];
            *p++ = a; *p++ = a>>8;
        }
        spidev_transfer(fc->spi, 0, sizeof(msg), msg);
    }
    move_free(m);
    if (fc->queued)
        fc->queued--;
    sendf("fpga_exec oid=%c executed=%c", fc->oid, 1);

    t->waketime = cur + fc->poll_ticks;
    return SF_RESCHEDULE;
}

/********************************************************************
 * Comandos acessíveis pelo host
 ********************************************************************/

// Configura novo controlador FPGA e envia parametros básicos
void
command_config_fpga(uint32_t *args)
{
    struct fpga_controller *fc = oid_alloc(args[0], command_config_fpga,
                                           sizeof(*fc));
    fc->oid = args[0];
    fc->timer.func = fpga_move_event;
    fc->spi = spidev_oid_lookup(args[1]);
    fc->trigger_pin = gpio_in_setup(args[2], 1);
    fc->poll_ticks = timer_from_us(10);
    move_queue_setup(&fc->mq, sizeof(struct fpga_pmove));
    fc->queued = 0;

    // Armazena ganhos PID por eixo para eventual depuração
    uint8_t idx = 3;
    for (uint8_t a = 0; a < 5; a++) {
        idx += 2; // pula oid e microsteps
    }
    for (uint8_t a = 0; a < 5; a++) {
        fc->pid_P[a] = args[idx++];
        fc->pid_I[a] = args[idx++];
        fc->pid_D[a] = args[idx++];
    }

    // Envia ao FPGA mensagem de configuração (conteúdo depende do hardware)
    uint8_t cfg[2] = { 0 }; // cmd=0 define configuração
    spidev_transfer(fc->spi, 0, sizeof(cfg), cfg);
}
DECL_COMMAND(command_config_fpga,
             "config_fpga oid=%c spi_oid=%c trigger_pin=%c"
             " axis_x_oid=%c microsteps_x=%hu"
             " axis_y_oid=%c microsteps_y=%hu"
             " axis_z_oid=%c microsteps_z=%hu"
             " axis_a_oid=%c microsteps_a=%hu"
             " axis_b_oid=%c microsteps_b=%hu"
             " pid_P_x=%hu pid_I_x=%hu pid_D_x=%hu"
             " pid_P_y=%hu pid_I_y=%hu pid_D_y=%hu"
             " pid_P_z=%hu pid_I_z=%hu pid_D_z=%hu"
             " pid_P_a=%hu pid_I_a=%hu pid_D_a=%hu"
             " pid_P_b=%hu pid_I_b=%hu pid_D_b=%hu");

// Enfileira novo movimento para o FPGA
void
command_queue_fpga_move(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    if (fc->queued >= FPGA_BUFFER_SIZE)
        shutdown("FPGA buffer full");
    struct fpga_move *m = move_alloc();
    m->type = 0;
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
    fc->timer.waketime = timer_read_time();
    sched_add_timer(&fc->timer);
}
DECL_COMMAND(command_queue_fpga_move,
             "queue_fpga_move oid=%c clock=%u interval=%u count=%hu add=%hi");

// Enfileira movimento paralelo com dados para todos os eixos
void
command_queue_fpga_pmove(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    if (fc->queued >= FPGA_BUFFER_SIZE)
        shutdown("FPGA buffer full");
    struct fpga_pmove *m = move_alloc();
    m->type = 1;
    m->waketime = args[1];
    uint8_t idx = 2;
    for (int i=0; i<5; i++) {
        m->interval[i] = args[idx++];
        m->count[i] = args[idx++];
        m->add[i] = args[idx++];
    }

    irq_disable();
    int need_add = move_queue_push(&m->node, &fc->mq);
    fc->queued++;
    irq_enable();
    if (!need_add)
        return;

    sched_del_timer(&fc->timer);
    fc->timer.waketime = timer_read_time();
    sched_add_timer(&fc->timer);
}
DECL_COMMAND(command_queue_fpga_pmove,
             "queue_fpga_pmove oid=%c clock=%u"
             " interval_x=%u count_x=%hu add_x=%hi"
             " interval_y=%u count_y=%hu add_y=%hi"
             " interval_z=%u count_z=%hu add_z=%hi"
             " interval_a=%u count_a=%hu add_a=%hi"
             " interval_b=%u count_b=%hu add_b=%hi");

// Consulta quantidade de espaços disponíveis no buffer
void
command_query_fpga_buffer(uint32_t *args)
{
    struct fpga_controller *fc = oid_lookup(args[0], command_config_fpga);
    uint8_t free = FPGA_BUFFER_SIZE - fc->queued;
    sendf("fpga_buffer oid=%c free=%c", args[0], free);
}
DECL_COMMAND(command_query_fpga_buffer, "query_fpga_buffer oid=%c");

// Limpa filas durante o desligamento
void
fpga_shutdown(void)
{
    uint8_t i; struct fpga_controller *fc;
    foreach_oid(i, fc, command_config_fpga) {
        move_queue_clear(&fc->mq);
    }
}
DECL_SHUTDOWN(fpga_shutdown);