#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);
void stepper_set_fpga(uint8_t oid, uint8_t enable);

#endif // stepper.h