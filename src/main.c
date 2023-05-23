/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>   /* for timing services */

/* Debug define */
#define DEBUG

/* Size of stack area used by each thread*/
#define STACK_SIZE 1024

/* Button Thread config */
#define btnThreadPrio 1                             /**<Button Thread priority (default 1) */
#define btnThreadPeriod 200                         /**<Button Thread periodicity (in ms) */
K_THREAD_STACK_DEFINE(btnThreadStack, STACK_SIZE);  /* Create thread stack space */
struct k_thread btnThreadData;                      /**<Button Thread data structure */
k_tid_t btnThreadID;                                /**<Button Thread ID */
void btnThread(void *argA, void *argB, void *argC); /* Thread code prototypes */

/* Vectors fot all INPUTs and OUTPUTs */
const uint8_t leds_pins[] = {13,14,15,16};          /* Vector with pins where leds are connected */
const uint8_t buttons_pins[] = {11,12,24,25};       /* Vector with pins where buttons are connected */

/* Get node ID for GPIO0, which has leds and buttons */ 
#define GPIO0_NODE DT_NODELABEL(gpio0)

/* Now get the device pointer for GPIO0 */
static const struct device * gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);

struct miniData {
    uint8_t led[4];
    uint8_t buttonState[4];
    float temp;
} miniData;


void main(void)
{
	btnThreadID = k_thread_create(&btnThreadData, btnThreadStack,
        K_THREAD_STACK_SIZEOF(btnThreadStack), btnThread,
        NULL, NULL, NULL, btnThreadPri, 0, K_NO_WAIT);

    return;
}

void btnThread(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */
    const struct gpio_dt_spec but0_dev = GPIO_DT_SPEC_GET(BUT0_NID, gpios); /* GPIO device structure for Button*/
        
    int ret=0;     /* Generic return value variable */
    
    /* Task init code */
    #ifdef DEBUG
        printk("Thread BTN init (periodic)\n");
    #endif
    
    /* Check if gpio0 device is ready */
	if (!device_is_ready(gpio0_dev)) { return; }

    for(i=0; i<sizeof(buttons_pins); i++) {
		ret = gpio_pin_configure(gpio0_dev, buttons_pins[i], GPIO_INPUT | GPIO_PULL_UP);
		if (ret < 0) { return; }
	}
           
    /* Compute next release instant */
    release_time = k_uptime_get() + btnThreadPeriod;

    /* Thread loop */
    while(1) {        
        
        #ifdef DEBUG
            printk("Thread BTN Activated\n\r");
        #endif  
        
        for(int i=0; i<sizeof(buttons_pins); i++){        
            if(BIT(buttons_pins[i]) & pins) {
                miniData->buttonState[i] = buttons_pins[i];
            }
        } 
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += btnThreadPeriod;

        }
    }

    /* Stop timing functions */
    timing_stop();
}

void ledThread(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */
    const struct gpio_dt_spec but0_dev = GPIO_DT_SPEC_GET(BUT0_NID, gpios); /* GPIO device structure for Button*/
        
    int ret=0;     /* Generic return value variable */
    
    /* Task init code */
    #ifdef DEBUG
        printk("Thread LED init (periodic)\n");
    #endif
    
    /* Check if gpio0 device is ready */
	if (!device_is_ready(gpio0_dev)) { return; }

    for(i=0; i<sizeof(leds_pins); i++) {
		ret = gpio_pin_configure(gpio0_dev, leds_pins[i], GPIO_OUTPUT);
		if (ret < 0) { return; }
	}
           
    /* Compute next release instant */
    release_time = k_uptime_get() + btnThreadPeriod;

    /* Thread loop */
    while(1) {        
        
        #ifdef DEBUG
            printk("Thread LED Activated\n\r");
        #endif  
        
        for(int i=0; i<sizeof(buttons_pins); i++){        
            gpio_pin_set(gpio0_dev, leds_pins[i], miniData->led[i]);
        } 
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += btnThreadPeriod;

        }
    }

    /* Stop timing functions */
    timing_stop();
}