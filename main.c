/*******************************************************************************
* File Name:   main.c
*
* Description: This code is intended to provide a external watchdog routine for
*  a Test PSoc board.
*
* Related Document: See README.md
*
*
********************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)

// Real time clock
#define RTC_INTERRUPT_PRIORITY (6u)
#define RTC_TIMER_CLOCK_HZ     (10000)
#define RTC_TIMER_PERIOD       (9) // 1ms precision

#define ALIVE_IN 	(P10_2)
#define RESET_OUT 	(P9_2)

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile uint64_t rtc_ms = 0;
volatile uint32_t alive_cnt = 0;
cyhal_gpio_callback_data_t gpio_alive_callback_data;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

/* Timer object used for real time clock */
cyhal_timer_t rtc_timer;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void initPeripherals();
void initTimer(void);
void restartTestPsoc();
void UART_wbyte(uint8_t tx_data);
void UART_wstring(char *str_ptr, size_t str_size);

/*******************************************************************************
* Function Definitions
*******************************************************************************/


/*******************************************************************************
* User defined error handling function
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
*  System monitoring
*******************************************************************************/
int main(void)
{
	enum listen_fsm_st
	{
		FSM_IDLE_ST,
		FSM_READ_CMD_ST
	};

	uint64_t alive_timer = 0,
			 serial_timer = 0,
			 time_diff = 0;

    initPeripherals();
    initTimer();

    int fsm_st = 0;
    while(1)
    {
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1) == CY_RSLT_SUCCESS)
        {
			switch (fsm_st) {
				// Idle state
				case FSM_IDLE_ST:
					// New command from host
					if (uart_read_value == 'W')
					{
						fsm_st = FSM_READ_CMD_ST;
					}
					break;

				// Read command state
				case FSM_READ_CMD_ST:
					// Reset DUT command
					if(uart_read_value == 'R')
					{
						restartTestPsoc();
					}
					fsm_st = FSM_IDLE_ST;
					break;

				// Invalid state
				default:
					fsm_st = FSM_IDLE_ST;
			}
        }

        time_diff = rtc_ms - alive_timer;
		if(time_diff >= 100)
		{
			if (alive_cnt == 0)
			{
				UART_wstring("WT", 2);		 // Signal alive timeout
				cyhal_system_delay_ms(3E3);
			}
			//printf("%lu\r\n", alive_cnt);
			alive_cnt = 0;
			alive_timer = rtc_ms;
		}


        time_diff = rtc_ms - serial_timer;
		if(time_diff >= 7E3)
		{
			cyhal_gpio_toggle(CYBSP_USER_LED);	// Visual alive
			UART_wstring("WA", 2); 				// Serial alive
			serial_timer = rtc_ms;
		}



    } // for loop
}


/*******************************************************************************
*   RTC interrupt handler - Real time clock.
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
	rtc_ms++;
}

/*******************************************************************************
*   GPIO interrupt handler.
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	alive_cnt++;
}

/*******************************************************************************
*   Transmit a byte via UART
*******************************************************************************/
void UART_wbyte(uint8_t tx_data)
{
	cy_rslt_t result;

	result = cyhal_uart_putc(&cy_retarget_io_uart_obj, tx_data);
	handle_error(result);

}

/*******************************************************************************
*   Transmit a string via UART
*******************************************************************************/
void UART_wstring(char *str_ptr, size_t str_size)
{
	for (uint32_t i = 0; i < str_size; i++)
	{
		UART_wbyte((uint8_t) str_ptr[i]);
	}
}

/*******************************************************************************
*   Generates a pulse to restart the Test PSoc board.
*******************************************************************************/
void restartTestPsoc()
{
    cyhal_gpio_write(RESET_OUT, 0u);
    cyhal_system_delay_ms(10);
    cyhal_gpio_write(RESET_OUT, 1u);
}

/*******************************************************************************
* This function creates and configures a Timer object.
*******************************************************************************/
 void initTimer(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t rtc_timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = RTC_TIMER_PERIOD,   		/* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&rtc_timer, NC, NULL);
    handle_error(result);

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&rtc_timer, &rtc_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&rtc_timer, RTC_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&rtc_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&rtc_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, RTC_INTERRUPT_PRIORITY, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&rtc_timer);
 }

/*******************************************************************************
*   Initialize needed peripherals.
*******************************************************************************/
void initPeripherals()
{
	cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    handle_error(result);

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            							CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    handle_error(result);

    /* Initialize the user LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
    							CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    handle_error(result);

    /* Initialize  ALIVE signal port*/
    result = cyhal_gpio_init(ALIVE_IN, CYHAL_GPIO_DIR_INPUT,
    							CYHAL_GPIO_DRIVE_PULLDOWN, 0u);
    handle_error(result);

    /* Initialize  RESET signal port*/
    result = cyhal_gpio_init(RESET_OUT, CYHAL_GPIO_DIR_OUTPUT,
    							CYHAL_GPIO_DRIVE_STRONG, 1u);
    handle_error(result);

    /* Configure GPIO interrupt for ALIVE signal*/
    gpio_alive_callback_data.callback = gpio_interrupt_handler;
    cyhal_gpio_register_callback(ALIVE_IN, &gpio_alive_callback_data);
    cyhal_gpio_enable_event(ALIVE_IN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

    /* Enable global interrupts */
    __enable_irq();
}

/* [] END OF FILE */
