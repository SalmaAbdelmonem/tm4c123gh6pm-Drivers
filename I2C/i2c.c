/**************************************************************************************************
 *
 * Module: I2C
 *
 * File Name: i2c.c
 *
 * Description: Source file for the I2C tm4c123gh6pm driver
 *
 **************************************************************************************************/

#include "i2c.h"

/**************************************************************************************************
 *                                 Interrupt Service Routines                                     *
 *************************************************************************************************/

void I2C_Handler(void)
{
    if(g_callBackPtr_I2C != NULL_PTR)
    {
        /* Call the Call Back function in the application  */
        (*g_callBackPtr_I2C)();
    }
}

/***************************************************************************************************
 *                                     Functions Definitions                                       *
 **************************************************************************************************/

/***************************************************************************************************
 * [Function Name]:   I2C_initSlave
 *
 * [Description]:     Function to initialize I2C module as a slave
 *
 * [Arguments]:       Slave address
 *
 * [Returns]:         void
 ***************************************************************************************************/

void I2C_initSlave(const uint8 slave_address)
{

    /*Enable clock for I2C module*/
    SYSCTL_RCGCI2C_R    |= SYSCTL_RCGCI2C_R1;
    /* Enable port A and provide clock */
    SYSCTL_RCGCGPIO_R   |= (1<<0);
    /* Enable pull up res for pin 6&7 */
    GPIO_PORTA_PUR_R    |= ((1<<6) | (1<<7));
    /* Enable digital functionality for pin 6&7 */
    GPIO_PORTA_DEN_R    |= ((1<<6) | (1<<7));
    /* Enable alternate function for pins 6&7 to be used by peripherals */
    GPIO_PORTA_AFSEL_R  |= ((1<<6) | (1<<7));
    /* Clear functions of pin 6&7 in PCTL register */
    GPIO_PORTA_PCTL_R   &= ~(GPIO_PCTL_PA7_M | GPIO_PCTL_PA6_M);
    /* Selects the specific peripheral signal for each GPIO pin when using AFSEL */
    GPIO_PORTA_PCTL_R   |= (GPIO_PCTL_PA6_I2C1SCL | GPIO_PCTL_PA7_I2C1SDA);
    /* Enable open drain functionality for data pins */
    GPIO_PORTA_ODR_R     = (1<<7);

    /* Enable I2C slave function in I2C module */
    I2C1_MCR_R  |= I2C_MCR_SFE;
    /* Set address of slave */
    I2C1_SOAR_R  = (I2C_SOAR_OAR_M & slave_address);
    /* Enable interrupts for I2C in interrupt controller */
    NVIC_EN1_R |= NVIC_EN1_INT_M;
    /* Enable sending interrupt to the interrupt controller */
    I2C1_SIMR_R |= I2C_SIMR_DATAIM;
    /* Clear data interrupt flag */
    I2C1_SICR_R |= I2C_SICR_DATAIC;

    /* Enables I2C slave operation */
    I2C1_SCSR_R |= I2C_SCSR_DA;
}

/***************************************************************************************************
 * [Function Name]:   I2C_Slave_sendByte
 *
 * [Description]:     Function to transmit a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void I2C_Slave_sendByte()
{
    /* Clear data interrupt flag */
    I2C1_SICR_R |= I2C_SICR_DATAIC;
    /* Place data to be transmitted in Slave data Register */
    I2C1_SDR_R = i2c_dataT;
}

/***************************************************************************************************
 * [Function Name]:   I2C_Slave_receiveByte
 *
 * [Description]:     Function to receive a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void I2C_Slave_receiveByte(void)
{

    /* Clear data interrupt flag */
    I2C1_SICR_R |= I2C_SICR_DATAIC;
    /* Place data received in a global variable */
    i2c_dataR = I2C1_SDR_R;
}

/***************************************************************************************************
 * [Function Name]:   I2C_setCallBack
 *
 * [Description]:     Function to set the Call Back function address
 *
 * [Arguments]:       pointer to function
 *
 * [Returns]:         void
 ***************************************************************************************************/

void I2C_setCallBack(void(*t_ptr)(void))
{
    /* Save the address of the Call back function in a global variable */
    g_callBackPtr_I2C = t_ptr;
}

/***************************************************************************************************
 * [Function Name]:   I2C_checkTransitionSlave
 *
 * [Description]:     Function to check if data in SDR register will be sent or received
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/

void I2C_checkTransitionSlave(void)
{
    /* Check transmission flag */
    if( (I2C1_SCSR_R & I2C_SCSR_TREQ) == I2C_SCSR_TREQ )
    {
        /* Transmit the required data */
        I2C_Slave_sendByte();
    }
    /* Check reception flag */
    else if( (I2C1_SCSR_R & I2C_SCSR_RREQ) == I2C_SCSR_RREQ )
    {
        /* Save the received data */
        I2C_Slave_receiveByte();
    }
}

/**************************************************************************************************/
