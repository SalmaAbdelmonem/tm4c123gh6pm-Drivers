/*************************************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.h
 *
 * Description: Source file for the UART tm4c123gh6pm driver
 *
 *************************************************************************************************/

#include "UART.h"

/**************************************************************************************************
 *                                 Interrupt Service Routines                                     *
 *************************************************************************************************/

void UART_Handler(void)
{
    if (g_callBackPtr_UART != NULL_PTR)
    {
        /* Call the Call Back function in the application  */
        (*g_callBackPtr_UART)();
    }
}

/***************************************************************************************************
 *                                     Functions Definitions                                       *
 **************************************************************************************************/

/***************************************************************************************************
 * [Function Name]:   UART_init
 *
 * [Description]:     Function to initialize UART module
 *
 * [Arguments]:       UART baudrate
 *
 * [Returns]:         void
 ***************************************************************************************************/

void UART_init( UART_BAUDRATE baudrate)
{
    /*      PC6 -> RX & PC7 -> TX     */

    /* Activate UART3 */
    SET_BIT(SYSCTL_RCGCUART_R, UART3);
    /* Enable port C and provide clock */
    SYSCTL_RCGCGPIO_R |= (1<<2);
    /* Enable digital mode in PC6 & PC7 */
    GPIO_PORTC_DEN_R |= ((1<<6) | (1<<7));
    /* Enable Alternate Function */
    GPIO_PORTC_AFSEL_R |= ((1<<6) | (1<<7));
    /* To use PC6 and PC7 as UART */
    GPIO_PORTC_PCTL_R |= (GPIO_PCTL_PC7_U3TX | GPIO_PCTL_PC6_U3RX);

    /* Disable UART3 */
    UART3_CTL_R = 0;
    /* Writing Integer Baudrate Divisor */
    UART3_IBRD_R = BAUD_IDIV(baudrate);
    /* Fractional Integer Baudrate Divisor */
    UART3_FBRD_R = BAUD_FDIV(baudrate);
    /* Clock Source is System Clock */
    UART3_CC_R = 0;
    /* Set parameters and Enable FIFO operation */
    UART3_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    /* Set FIFO to 4bytes */
    UART3_IFLS_R = UART_IFLS_RX2_8;
    /* Clear receive and transmit interrupts */
    UART3_ICR_R |= (UART_ICR_TXIC | UART_ICR_RXIC);
    /* Enable UART3 Rx & Tx interrupt */
    UART3_IM_R = (UART_IM_TXIM | UART_IM_RXIM);
    /* Enable interrupts for UART in interrupt controller */
    NVIC_EN1_R |= NVIC_EN1_INT_M;

    /* Enable UART3, TXE & RXE */
    UART3_CTL_R = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
}

/***************************************************************************************************
 * [Function Name]:   UART_sendByte
 *
 * [Description]:     Function to transmit a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void UART_sendByte(void)
{
    /* Clear transmission and reception flags */
    UART3_ICR_R |= (UART_ICR_TXIC | UART_ICR_RXIC);
    /* Place data to be sent into Data Register */
    UART3_DR_R = uart_dataT;
}

/***************************************************************************************************
 * [Function Name]:   UART_receiveByte
 *
 * [Description]:     Function to receive a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void UART_receiveByte(void)
{
    /* Clear transmission and reception flags */
    UART3_ICR_R |= (UART_ICR_TXIC | UART_ICR_RXIC);
    /* Save the received data into a global variable */
    uart_dataR = (uint8) UART3_DR_R;
}

/***************************************************************************************************
 * [Function Name]:   UART_data
 *
 * [Description]:     Function to collect 4bytes data in an array
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void UART_data(void)
{
    uint8 i=0;
    for(i=0;i<4;i++)
    {
        UART_receiveByte();
        dataR[i] = uart_dataR;
    }
}

/***************************************************************************************************
 * [Function Name]:   UART_setCallBack
 *
 * [Description]:     Function to set the Call Back function address
 *
 * [Arguments]:       Pointer to function
 *
 * [Returns]:         void
 ***************************************************************************************************/

void UART_setCallBack(void (*t_ptr)(void))
{
    /* Save the address of the Call back function in a global variable */
    g_callBackPtr_UART = t_ptr;
}

/***************************************************************************************************/
