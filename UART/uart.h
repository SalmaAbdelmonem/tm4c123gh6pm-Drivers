/*************************************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.h
 *
 * Description: Header file for the UART tm4c123gh6pm driver
 *
 *************************************************************************************************/

#ifndef UART_H_
#define UART_H_

#include "inc/std_types.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/common_macros.h"

/**************************************************************************************************
 *                                      Global Variables                                         *
 *************************************************************************************************/
/* Global variables to hold the address of the call back function in the application */
static volatile void (*g_callBackPtr_UART)(void) = NULL_PTR;

/* Global variables to save Received and transmitted data */
volatile uint8 uart_dataT;
volatile uint8 uart_dataR;

/* Global array to save Received 4bytes data */
volatile uint8 dataR[4];

/*************************************************************************************************
 *                                        Definitions                                            *
 *************************************************************************************************/
#define CLK_SPEED	    16e6
#define BAUD_IDIV(B)	(int)(CLK_SPEED/(16*B))
#define BAUD_FDIV(B) 	(int)((((float)CLK_SPEED/(16*B)) - BAUD_IDIV(B))*64 + 0.5)

#define UART0   0
#define UART1   1
#define UART2   2
#define UART3   3
#define UART4   4
#define UART5   5
#define UART6   6
#define UART7   7

/************************************************************************************************
 *                                        Types Declaration                                      *
 *************************************************************************************************/

typedef enum UART_BAUDRATE{
	UART_BAUD_300 		    = 300,
	UART_BAUD_1200 			= 1200,
	UART_BAUD_2400 			= 2400,
	UART_BAUD_4800 			= 4800,
	UART_BAUD_9600 			= 9600,
	UART_BAUD_19200 		= 19200,
	UART_BAUD_38400 		= 38400,
	UART_BAUD_57600 		= 57600,
	UART_BAUD_74880 		= 74880,
	UART_BAUD_115200		= 115200,
} UART_BAUDRATE;

/**************************************************************************************************
 *                                 Interrupt Service Routines                                     *
 *************************************************************************************************/
void UART_Handler(void);

/*************************************************************************************************
 *                                     Functions Prototypes                                      *
 ************************************************************************************************/
/***************************************************************************************************
 * [Function Name]:   UART_init
 *
 * [Description]:     Function to initialize UART module
 *
 * [Arguments]:       UART baudrate
 *
 * [Returns]:         void
 ***************************************************************************************************/
void UART_init( UART_BAUDRATE baudrate);

/***************************************************************************************************
 * [Function Name]:   UART_sendByte
 *
 * [Description]:     Function to transmit a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void UART_sendByte(void);

/***************************************************************************************************
 * [Function Name]:   UART_receiveByte
 *
 * [Description]:     Function to receive a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void UART_receiveByte(void);

/***************************************************************************************************
 * [Function Name]:   UART_FIFO
 *
 * [Description]:     Function to collect 4bytes data in an array
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void UART_data(void);

/***************************************************************************************************
 * [Function Name]:   UART_setCallBack
 *
 * [Description]:     Function to set the Call Back function address
 *
 * [Arguments]:       Pointer to function
 *
 * [Returns]:         void
 ***************************************************************************************************/
void UART_setCallBack(void(*t_ptr)(void));

/***************************************************************************************************/

#endif
