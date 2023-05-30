/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>          /* for kernel functions*/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>   /* for timing services */
#include <zephyr/drivers/i2c.h>

/* Debug define */
#define DEBUG

/* Size of stack area used by each thread*/
#define STACK_SIZE 1024

/* Button Thread config */
#define btnThreadPrio 1                             /**<Button Thread priority (default 1) */
#define btnThreadPeriod 1000                        /**<Button Thread periodicity (in ms) */
K_THREAD_STACK_DEFINE(btnThreadStack, STACK_SIZE);  /* Create thread stack space */
struct k_thread btnThreadData;                      /**<Button Thread data structure */
k_tid_t btnThreadID;                                /**<Button Thread ID */
void btnThread(void *argA, void *argB, void *argC); /* Button Thread code prototypes */

/* Led Thread config */
#define ledThreadPrio 1                             /**<Led Thread priority (default 1) */
#define ledThreadPeriod 1000                        /**<Led Thread periodicity (in ms) */
K_THREAD_STACK_DEFINE(ledThreadStack, STACK_SIZE);  /* Create thread stack space */
struct k_thread ledThreadData;                      /**<Led Thread data structure */
k_tid_t ledThreadID;                                /**<Led Thread ID */
void ledThread(void *argA, void *argB, void *argC); /* Led Thread code prototypes */

/* I2C Thread config */
#define i2cThreadPrio 1                             /**<I2C Thread priority (default 1) */
#define i2cThreadPeriod 1000                        /**<I2C Thread periodicity (in ms) */
K_THREAD_STACK_DEFINE(i2cThreadStack, STACK_SIZE);  /* Create thread stack space */
struct k_thread i2cThreadData;                      /**<I2C Thread data structure */
k_tid_t i2cThreadID;                                /**<I2C Thread ID */
void i2cThread(void *argA, void *argB, void *argC); /* I2C Thread code prototypes */

/* Vectors fot all INPUTs and OUTPUTs */
const uint8_t leds_pins[] = {13,14,15,16};          /* Vector with pins where leds are connected */
const uint8_t buttons_pins[] = {11,12,24,25};       /* Vector with pins where buttons are connected */

/* Get node ID for GPIO0, which has leds and buttons */ 
#define GPIO0_NODE DT_NODELABEL(gpio0)

/* Now get the device pointer for GPIO0 */
static const struct device * gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);

/* Get node ID for I2C0, which has the temperature sensor */
#define I2C0_NODE DT_NODELABEL(tempsensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

/**
 * @brief This structure contains all the data that is shared between the threads.
 *
 */ 
struct miniData {
    uint8_t led[4];
    uint8_t buttonState[4];
    float temp;
} miniData;


/**
 * @brief Main Function where all the threads are initialized.
 *
 */ 
void main(void)
{
    /* Create the Button Thread */
	/*btnThreadID = k_thread_create(&btnThreadData, btnThreadStack,
        K_THREAD_STACK_SIZEOF(btnThreadStack), btnThread,
        NULL, NULL, NULL, btnThreadPrio, 0, K_NO_WAIT);*/

    i2cThreadID = k_thread_create(&i2cThreadData, i2cThreadStack,
        K_THREAD_STACK_SIZEOF(i2cThreadStack), i2cThread,
        NULL, NULL, NULL, i2cThreadPrio, 0, K_NO_WAIT);

    return;
}

/**
 * @brief Button Thread where the button state is read and stored in the shared structure.
 *
 */ 
void btnThread(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */
        
    int ret=0;     /* Generic return value variable */
    
    /* Task init code */
    #ifdef DEBUG
        printk("Thread BTN init (periodic)\n");
    #endif
    
    /* Check if gpio0 device is ready */
	if (!device_is_ready(gpio0_dev)) { return; }

    for(int i=0; i<sizeof(buttons_pins); i++) {
		ret = gpio_pin_configure(gpio0_dev, buttons_pins[i], GPIO_INPUT | GPIO_PULL_UP);
		if (ret < 0) { return; }
	}

    uint8_t pinmask = 0;
	for(int i=0; i<sizeof(buttons_pins); i++) {
		pinmask |= BIT(buttons_pins[i]);
	}
           
    /* Compute next release instant */
    release_time = k_uptime_get() + btnThreadPeriod;

    /* Thread loop */
    while(1) {        
        
        #ifdef DEBUG
            printk("Thread BTN Activated\n\r");
        #endif  
        
        for(int i=0; i<sizeof(buttons_pins); i++){        
            if(BIT(buttons_pins[i]) & pinmask) {
                miniData.buttonState[i] = buttons_pins[i];
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

/**
 * @brief Led Thread where the leds are set according to the variable stored in the shared structure.
 *
 */ 
void ledThread(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */
        
    int ret=0;     /* Generic return value variable */
    
    /* Task init code */
    #ifdef DEBUG
        printk("Thread LED init (periodic)\n");
    #endif
    
    /* Check if gpio0 device is ready */
	if (!device_is_ready(gpio0_dev)) { return; }

    for(int i=0; i<sizeof(leds_pins); i++) {
		ret = gpio_pin_configure(gpio0_dev, leds_pins[i], GPIO_OUTPUT);
		if (ret < 0) { return; }
	}
           
    /* Compute next release instant */
    release_time = k_uptime_get() + ledThreadPeriod;

    /* Thread loop */
    while(1) {        
        
        #ifdef DEBUG
            printk("Thread LED Activated\n\r");
        #endif  
        
        for(int i=0; i<sizeof(leds_pins); i++){        
            gpio_pin_set(gpio0_dev, leds_pins[i], miniData.led[i]);
        } 
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += ledThreadPeriod;

        }
    }

    /* Stop timing functions */
    timing_stop();
}

/**
 * @brief I2C Thread where the temperature is read and stored in the shared structure.
 *
 */ 
void i2cThread(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */
        
    int ret=0;     /* Generic return value variable */
    
    /* Task init code */
    #ifdef DEBUG
        printk("Thread I2C init (periodic)\n");
    #endif
    
    if (!device_is_ready(dev_i2c.bus)) { return; }
    uint8_t data;

    /* Write via I2C to make the Temp Sensor go into normal mode */
    uint8_t config = 0x00;
    ret = i2c_write_dt(&dev_i2c, &config, sizeof(config));
    if(ret != 0){
	    printk("Failed to write to I2C device address %x at reg. %x \n", dev_i2c.addr,config);
    }

    /* Compute next release instant */
    release_time = k_uptime_get() + i2cThreadPeriod;

    /* Thread loop */
    while(1) {        
        
        #ifdef DEBUG
            printk("Thread I2C Activated\n\r");
        #endif  

        /* Read via I2C the temperature */
        ret = i2c_read_dt(&dev_i2c, &data, sizeof(data));
        if (ret < 0) { return; }
        miniData.temp = data;
        #ifdef DEBUG
            printk("Temperature: %d\n\r", data);
        #endif
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += i2cThreadPeriod;
        }
    }

    /* Stop timing functions */
    timing_stop();
}