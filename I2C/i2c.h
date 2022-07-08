/*************************************************************************************************
 *
 * Module: I2C
 *
 * File Name: i2c.h
 *
 * Description: Header file for the I2C tm4c123gh6pm driver
 *
 *************************************************************************************************/

#ifndef I2C_H_
#define I2C_H_

#include "inc/tm4c123gh6pm.h"
#include "inc/std_types.h"

/**************************************************************************************************
 *                                      Global Variables                                         *
 *************************************************************************************************/

/* Global variables to hold the address of the call back function in the application */
static volatile void (*g_callBackPtr_I2C)(void) = NULL_PTR;

/* Global variables to save Received and transmitted data */
volatile uint8 i2c_dataR;
volatile uint8 i2c_dataT;

/**************************************************************************************************
 *                                 Interrupt Service Routines                                     *
 *************************************************************************************************/

void I2C_Handler(void);

/*************************************************************************************************
 *                                     Functions Prototypes                                      *
 ************************************************************************************************/

/***************************************************************************************************
 * [Function Name]:   I2C_initSlave
 *
 * [Description]:     Function to initialize I2C module as a slave
 *
 * [Arguments]:       Slave address
 *
 * [Returns]:         void
 ***************************************************************************************************/
void  I2C_initSlave(uint8 slave_address);

/***************************************************************************************************
 * [Function Name]:   I2C_Slave_sendByte
 *
 * [Description]:     Function to transmit a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void  I2C_Slave_sendByte();

/***************************************************************************************************
 * [Function Name]:   I2C_Slave_receiveByte
 *
 * [Description]:     Function to receive a byte
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void  I2C_Slave_receiveByte(void);

/***************************************************************************************************
 * [Function Name]:   I2C_setCallBack
 *
 * [Description]:     Function to set the Call Back function address
 *
 * [Arguments]:       Pointer to function
 *
 * [Returns]:         void
 ***************************************************************************************************/
void I2C_setCallBack(void(*t_ptr)(void));

/***************************************************************************************************
 * [Function Name]:   I2C_checkTransitionSlave
 *
 * [Description]:     Function to check if data in SDR register will be sent or received
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 ***************************************************************************************************/
void I2C_checkTransitionSlave(void);

/***********************************************************************************************/
#endif /* I2C_H_ */
