/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************

Changes made by Luckas Farias on Hackathon from Motorola 

Project: MotoKey

******************************************************/

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/board/mods.h>

#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>

#include "stm32_tim.h"

#define BLINKY_ACTIVITY    10
#define BLINKY_TIM          6
#define BLINKY_TIM_FREQ  1000
#define BLINKY_PERIOD    1000

#define LED_ON              0
#define LED_OFF             1

static struct stm32_tim_dev_s *tim_dev_close;
static struct stm32_tim_dev_s *tim_dev_open_car;
static struct stm32_tim_dev_s *tim_dev_open_bag;


// Function that make the Signal from Open Car blink

static int blinky_timer_handler_open_car(int irq, FAR void *context)
{
    uint8_t new_val;

    pm_activity(BLINKY_ACTIVITY);
    STM32_TIM_ACKINT(tim_dev_open_car, 0);

/****************************************************************/
    new_val = gpio_get_value(GPIO_MODS_LED_DRV_1) ^ 1; // Change the I/O
    gpio_set_value(GPIO_MODS_LED_DRV_1, new_val);
/****************************************************************/

    llvdbg("new_val_open_car=%d\n", new_val);
    return 0;
}

static void blinky_open_car(void)
{
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);

    if (!tim_dev_open_car) {
        dbg("BLINKY OPEN THE CAR\n");

        tim_dev_open_car = stm32_tim_init(BLINKY_TIM);

        DEBUGASSERT(tim_dev_open_car);

        STM32_TIM_SETPERIOD(tim_dev_open_car, BLINKY_PERIOD);
        STM32_TIM_SETCLOCK(tim_dev_open_car, BLINKY_TIM_FREQ);
        STM32_TIM_SETMODE(tim_dev_open_car, STM32_TIM_MODE_PULSE);
        STM32_TIM_SETISR(tim_dev_open_car, blinky_timer_handler_open_car, 0);
        STM32_TIM_ENABLEINT(tim_dev_open_car, 0);
    } else {
        dbg("ignore the Open car\n");
    }
}

// Function that make the Signal from Open Bag dor blink

static int blinky_timer_handler_open_bag(int irq, FAR void *context)
{
    uint8_t new_val;

    pm_activity(BLINKY_ACTIVITY);
    STM32_TIM_ACKINT(tim_dev_open_bag, 0);

/****************************************************************/
    new_val = gpio_get_value(GPIO_MODS_LED_DRV_2) ^ 1; // Change the I/O
    gpio_set_value(GPIO_MODS_LED_DRV_2, new_val);
/****************************************************************/

    llvdbg("new_val_open_bag=%d\n", new_val);
    return 0;
}

static void blinky_open_bag(void)
{
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);

    if (!tim_dev_open_bag) {
        dbg("BLINKY OPEN BAG \n");

        tim_dev_open_bag = stm32_tim_init(BLINKY_TIM);

        DEBUGASSERT(tim_dev_open_bag);

        STM32_TIM_SETPERIOD(tim_dev_open_bag, 300 ); // BLINKY_PERIOD); 
// We think that 1 second for interval is too long, 
// now it will be on by 330, off by 330 and on again by 330... 
// This will made it easy to open the bag from the car
        STM32_TIM_SETCLOCK(tim_dev_open_bag, BLINKY_TIM_FREQ);
        STM32_TIM_SETMODE(tim_dev_open_bag, STM32_TIM_MODE_PULSE);
        STM32_TIM_SETISR(tim_dev_open_bag, blinky_timer_handler_open_bag, 0);
        STM32_TIM_ENABLEINT(tim_dev_open_bag, 0);
    } else {
        dbg("ignore the bag open\n");
    }
}

// Function that make the Signal from Close dor blink

static int blinky_timer_handler_close(int irq, FAR void *context)
{
    uint8_t new_val;

    pm_activity(BLINKY_ACTIVITY);
    STM32_TIM_ACKINT(tim_dev_close, 0);

/****************************************************************/
    new_val = gpio_get_value(GPIO_MODS_LED_DRV_3) ^ 1; // Change the I/O
    gpio_set_value(GPIO_MODS_LED_DRV_3, new_val);
/****************************************************************/

    llvdbg("new_val_close=%d\n", new_val);
    return 0;
}

static void blinky_close(void)
{
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);

    if (!tim_dev_close) {
        dbg("BLINKY CLOSE THE CAR\n");

        tim_dev_close = stm32_tim_init(BLINKY_TIM);

        DEBUGASSERT(tim_dev_close);

        STM32_TIM_SETPERIOD(tim_dev_close, BLINKY_PERIOD);
        STM32_TIM_SETCLOCK(tim_dev_close, BLINKY_TIM_FREQ);
        STM32_TIM_SETMODE(tim_dev_close, STM32_TIM_MODE_PULSE);
        STM32_TIM_SETISR(tim_dev_close, blinky_timer_handler_close, 0);
        STM32_TIM_ENABLEINT(tim_dev_close, 0);
    } else {
        dbg("ignore close the car\n");
    }
}

// Module that stops the 3 signals from control
static void blinky_stop(void)
{
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 0); // What it is?

    if (tim_dev_open_car) { // If is a signal for open the car
        dbg("STOP ALL SIGNALs\n");

	// Stop the signal that open the car
        STM32_TIM_DISABLEINT(tim_dev_open_car, 0);
        stm32_tim_deinit(tim_dev_open_car);
        tim_dev_open_car = NULL;

	// Stop the signal that open the bag
        STM32_TIM_DISABLEINT(tim_dev_open_bag, 0);
        stm32_tim_deinit(tim_dev_open_bag);
        tim_dev_open_bag = NULL;

	// Stop the signal that close the car
        STM32_TIM_DISABLEINT(tim_dev_close, 0);
        stm32_tim_deinit(tim_dev_close);
        tim_dev_close = NULL;

/****************************************************************/
        gpio_set_value(GPIO_MODS_LED_DRV_1, LED_OFF); // CHANGE I/O
        gpio_set_value(GPIO_MODS_LED_DRV_2, LED_OFF); // CHANGE I/O
        gpio_set_value(GPIO_MODS_LED_DRV_3, LED_OFF); // CHANGE I/O
/****************************************************************/
    } else {
        dbg("ALL Signals already Stoped\n");
    }
}


// Module that will recive the code from app to send to a pin
static int blinky_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    if (len == 0)
        return -EINVAL;

    if (data[0] == 0 || data[0] == '0')
        blinky_close();
    else if (data[0] == 1 || data[0] == '1')
        blinky_open_bag();
    else if (data[0] == 2 || data[0] == '2')
        blinky_open_car();
    else  
        blinky_stop();
	
	
    return 0;
}

static int blinky_register_callback(struct device *dev,
                                    raw_send_callback callback)
{
    /* Nothing to do */
    return 0;
}

static int blinky_unregister_callback(struct device *dev)
{
    /* Nothing to do */
    return 0;
}

static int blinky_probe(struct device *dev)
{
/****************************************************************/ // OPEN CAR
    gpio_direction_out(GPIO_MODS_LED_DRV_1, LED_OFF);
    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);
/****************************************************************/ // OPEN BAG
    gpio_direction_out(GPIO_MODS_LED_DRV_2, LED_OFF);
    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);
/****************************************************************/ // CLOSE
    gpio_direction_out(GPIO_MODS_LED_DRV_3, LED_OFF);
    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);
/****************************************************************/
    return 0;
}

static struct device_raw_type_ops blinky_type_ops = {
    .recv = blinky_recv,
    .register_callback = blinky_register_callback,
    .unregister_callback = blinky_unregister_callback,
};

static struct device_driver_ops blinky_driver_ops = {
    .probe = blinky_probe,
    .type_ops = &blinky_type_ops,
};

struct device_driver mods_raw_blinky_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_blinky",
    .desc = "Blinky LED Raw Interface",
    .ops = &blinky_driver_ops,
};
