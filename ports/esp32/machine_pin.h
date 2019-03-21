#ifndef MACHINE_PIN_H_
#define MACHINE_PIN_H_

#include "driver/gpio.h"
#include "py/obj.h"

typedef struct _machine_pin_obj_t {
    mp_obj_base_t base;
    gpio_num_t id;
} machine_pin_obj_t;

#endif /* MACHINE_PIN_H_ */