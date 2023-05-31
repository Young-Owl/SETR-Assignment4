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
#include <drivers/uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Debug define */
#define DEBUG

/* Error define */
#define FATAL_ERR -1                                /* Fatal error return code, app terminates */

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

/* UART Thread config and other configs */
#define uartThreadPrio 1                            /**<uart Thread priority (default 1) */
K_THREAD_STACK_DEFINE(uartThreadStack, STACK_SIZE); /* Create thread stack space */
struct k_thread uartThreadData;                     /**<uart Thread data structure */
k_tid_t uartThreadID;                               /**<uart Thread ID */
#define RXBUF_SIZE 20                               /* RX buffer size */
#define TXBUF_SIZE 20                               /* TX buffer size */
#define RX_TIMEOUT 1000                             /* Inactivity period after the instant when last char was received that triggers an rx event (in us) */
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};
void uartThread(void *argA, void *argB, void *argC);/* uart Thread code prototypes */

/* UART related variables */
const struct device *uart_dev;                      /* Pointer to device struct */ 
static uint8_t rx_buf[RXBUF_SIZE];                  /* RX buffer, to store received data */
static uint8_t rx_chars[RXBUF_SIZE];                /* Chars actually received  */
volatile int uart_rxbuf_nchar=0;                    /* Number of chars currnetly on the rx buffer */

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

/* Get node ID for UART */
#define UART_NODE DT_NODELABEL(uart0)              

/* Semaphore for task synch */
struct k_sem sem_uart;

/* UART callback function prototype */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);


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
 * @brief Main Function where all the threads and sempahores are initialized.
 *
 */ 
void main(void)
{
    /* Local vars */    
    int err=0; /* Generic error variable */
    uint8_t welcome_mesg[] = "UART demo: Type a few chars in a row and then pause for a little while ...\n\r"; 

    /* Bind to UART */
    uart_dev= device_get_binding(DT_LABEL(UART_NODE));
    if (uart_dev == NULL) {
        printk("device_get_binding() error for device %s!\n\r", DT_LABEL(UART_NODE));
        return;
    }
    else {
        printk("UART binding successful\n\r");
    }

    /* Configure UART */
    err = uart_configure(uart_dev, &uart_cfg);
    if (err == -ENOSYS) { /* If invalid configuration */
        printk("uart_configure() error. Invalid configuration\n\r");
        return; 
    }

    /* Register callback */
    err = uart_callback_set(uart_dev, uart_cb, NULL);
    if (err) {
        printk("uart_callback_set() error. Error code:%d\n\r",err);
        return;
    }
		
    /* Enable data reception */
    err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
    if (err) {
        printk("uart_rx_enable() error. Error code:%d\n\r",err);
        return;
    }

    /* Send a welcome message */ 
    /* Last arg is timeout. Only relevant if flow controll is used */
    err = uart_tx(uart_dev, welcome_mesg, sizeof(welcome_mesg), SYS_FOREVER_MS);
    if (err) {
        printk("uart_tx() error. Error code:%d\n\r",err);
        return;
    }
    
    /* Create and init semaphore */
    k_sem_init(&sem_uart, 0, 1);

    /* Create the Button Thread */
	/*btnThreadID = k_thread_create(&btnThreadData, btnThreadStack,
        K_THREAD_STACK_SIZEOF(btnThreadStack), btnThread,
        NULL, NULL, NULL, btnThreadPrio, 0, K_NO_WAIT);*/

    /* Create the I2C Thread */
    /*i2cThreadID = k_thread_create(&i2cThreadData, i2cThreadStack,
        K_THREAD_STACK_SIZEOF(i2cThreadStack), i2cThread,
        NULL, NULL, NULL, i2cThreadPrio, 0, K_NO_WAIT);*/

    /* Create the UART Thread*/
    uartThreadID = k_thread_create(&uartThreadData, uartThreadStack,
        K_THREAD_STACK_SIZEOF(uartThreadStack), uartThread,
        NULL, NULL, NULL, uartThreadPrio, 0, K_NO_WAIT);

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

void uartThread(void *argA , void *argB, void *argC)
{
    /* Local vars*/
    int err=0;        /* Generic error variable */
    uint8_t rep_mesg[TXBUF_SIZE];
    char c;

    #ifdef DEBUG
        printk("Thread uart init (sporadic)\n");
    #endif
    
    while(1) {
        k_sem_take(&sem_uart,  K_FOREVER);
        
        if(uart_rxbuf_nchar > 0) {
            rx_chars[uart_rxbuf_nchar] = 0; /* Terminate the string */

            sprintf(rep_mesg,"You typed [%s]\r",rx_chars);  
            
            c = rx_chars[uart_rxbuf_nchar-1];
           // printk("c = %c\n\r",c);
            printk("c = %x\n\r",c);
            printk("uart1 = %u\n\r",uart_rxbuf_nchar);

            if(c == 0xd) {
                for (int i = 0; i < uart_rxbuf_nchar; i++){
                    rx_chars[i] = 0;
                }
                uart_rxbuf_nchar = 0;
                printk("zz\n\r");
            }
            printk("uart2 = %u\n\r",uart_rxbuf_nchar);

            
            err = uart_tx(uart_dev, rep_mesg, strlen(rep_mesg), SYS_FOREVER_MS);
            if (err) {
                printk("uart_tx() error. Error code:%d\n\r",err);
                return;
            }
        }
        k_msleep(1);
    }
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int err;

    switch (evt->type) {
	
        case UART_TX_DONE:
		    //printk("UART_TX_DONE event \n\r");
            break;

    	case UART_TX_ABORTED:
	    	printk("UART_TX_ABORTED event \n\r");
		    break;
		
	    case UART_RX_RDY:
		    //printk("UART_RX_RDY event \n\r");
            /* Just copy data to a buffer. Usually it is preferable to use e.g. a FIFO to communicate with a task that shall process the messages*/
            memcpy(&rx_chars[uart_rxbuf_nchar],&(rx_buf[evt->data.rx.offset]),evt->data.rx.len); 
            uart_rxbuf_nchar++;       
            k_sem_give(&sem_uart);    
		    break;

	    case UART_RX_BUF_REQUEST:
		    printk("UART_RX_BUF_REQUEST event \n\r");
		    break;

	    case UART_RX_BUF_RELEASED:
		    printk("UART_RX_BUF_RELEASED event \n\r");
		    break;
		
	    case UART_RX_DISABLED: 
            /* When the RX_BUFF becomes full RX is is disabled automaticaly.  */
            /* It must be re-enabled manually for continuous reception */
            printk("UART_RX_DISABLED event \n\r");
		    err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
            if (err) {
                printk("uart_rx_enable() error. Error code:%d\n\r",err);
                exit(FATAL_ERR);                
            }
		    break;

	    case UART_RX_STOPPED:
		    printk("UART_RX_STOPPED event \n\r");
		    break;
		
	    default:
            printk("UART: unknown event \n\r");
		    break;
    }

}