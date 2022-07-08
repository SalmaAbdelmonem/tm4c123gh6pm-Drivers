/******************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm.h
 *
 * Description: Header file for the PWM tm4c123gh6pm driver
 *
 *
 *******************************************************************************/

#ifndef PWM_H_
#define PWM_H_

#include "inc/std_types.h"
#include "inc/common_macros.h"
#include "inc/tm4c123gh6pm.h"
#include <stdint.h>

/**************************************************************************************************
 *                                     Types Declaration                                           *
**************************************************************************************************/

typedef enum
{
    MODULE_0, MODULE_1
}Module_Id;

typedef enum
{
    GEN_0, GEN_1, GEN_2, GEN_3
}Generator_Id;

typedef enum
{
    F_CPU_2, F_CPU_4, F_CPU_8, F_CPU_16, F_CPU_32, F_CPU_64 = 7
}PWM_Clock;

typedef struct
{
    Module_Id module_id;
    Generator_Id gen_id;
    PWM_Clock clock;
}PWM_Config;

typedef struct
{
    Module_Id module_id;
    Generator_Id gen_id;
    uint16 load_value;
    float32 duty_cycle;
}PWM_Set;

/*************************************************************************************************
 *                                     Functions Prototypes                                      *
 ************************************************************************************************/

/*************************************************************************************************
 * [Function Name]:   PWM_Init
 *
 * [Description]:     Function to initialize the PWM modules.
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
/************************************************************************************************/

void PWM_Init(const PWM_Config * Config_Ptr);

/*/***********************************************************************************************
 * [Function Name]:   PWM_setValues
 *
 * [Description]:     Function to set load and compare value of PWM,
 * and pass generated signal through the chosen pin.
 *
 * [Arguments]:       Dynamic Configuration
 *
 * [Returns]:         void
 /***********************************************************************************************/

void PWM_setValues (const PWM_Set * Set_Ptr);

/*************************************************************************************************/

#endif
