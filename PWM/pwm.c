/**************************************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm.c
 *
 * Description: Source file for the PWM tm4c123gh6pm driver
 *
 **************************************************************************************************/

#include "pwm.h"

/**************************************************************************************************
 *                                  Functions Definitions                                         *
 **************************************************************************************************/
/***************************************************************************************************
 * [Function Name]:   PWM_Init
 *
 * [Description]:     Function to initialize the required PWM module.
 *
 * [Arguments]:       void
 *
 * [Returns]:         void
 **************************************************************************************************/

void PWM_Init (const PWM_Config * Config_Ptr)
{
    switch(Config_Ptr->module_id)
    {
    case MODULE_0:

        /* Activate PWM module according to input */
        SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
        /* Clear bits that responsible about clock divisor */
        SYSCTL_RCC_R &= ~(SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M);
        /* Enable PWM clock divisor */
        SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV ;
        /* Choose clock divisor */
        SYSCTL_RCC_R |= ((Config_Ptr->clock)<<17);

        switch(Config_Ptr->gen_id)
        {
        case GEN_0: //PB6

            /* Activate system clock to PORT */
            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
            /* Enable alternate function for PWM o/p pin */
            GPIO_PORTB_AFSEL_R |= (1<<6);
            /* Disable analog functionality on output PIN */
            GPIO_PORTB_AMSEL_R &= ~(1<<6);
            /* Enable digital functionality in output PIN */
            GPIO_PORTB_DEN_R |= (1<<6);
            /* Clear PWM Port PIN in port control register */
            GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB6_M);
            /* Set Pin as PWM output */
            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0;
            /* PWM control register configuration */
            PWM0_0_CTL_R = 0;
            /* PWM control pulses shape 'down -> load, 0&compA -> high */
            PWM0_0_GENA_R = (PWM_0_GENA_ACTCMPAD_M | PWM_0_GENA_ACTLOAD_ZERO);
            /* Enable PWM generation block */
            PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;
            /* Enable Passing PWM through pin */
            PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;
            break;

        case GEN_1: //PB4

            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
            GPIO_PORTB_AFSEL_R |= (1<<4);
            GPIO_PORTB_AMSEL_R &= ~(1<<4);
            GPIO_PORTB_DEN_R |= (1<<4);
            GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB4_M);
            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2;
            PWM0_1_CTL_R = 0;
            PWM0_1_GENA_R = (PWM_0_GENA_ACTCMPAD_M | PWM_0_GENA_ACTLOAD_ZERO);
            PWM0_1_CTL_R |= PWM_0_CTL_ENABLE;
            PWM0_ENABLE_R |= PWM_ENABLE_PWM2EN;
            break;

        case GEN_2: //PE4

            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
            GPIO_PORTE_AFSEL_R |= (1<<4);
            GPIO_PORTE_AMSEL_R &= ~(1<<4);
            GPIO_PORTE_DEN_R |= (1<<4);
            GPIO_PORTE_PCTL_R &= ~(GPIO_PCTL_PE4_M);
            GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4;
            PWM0_2_CTL_R = 0;
            PWM0_2_GENA_R = (PWM_0_GENA_ACTCMPAD_M | PWM_0_GENA_ACTLOAD_ZERO);
            PWM0_2_CTL_R |= PWM_0_CTL_ENABLE;
            PWM0_ENABLE_R |= PWM_ENABLE_PWM4EN;
            break;

        case GEN_3: //PC4

            GPIO_PORTC_AFSEL_R |= (1<<4);
            GPIO_PORTC_AMSEL_R &= ~(1<<4);
            GPIO_PORTC_DEN_R |= (1<<4);
            GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M);
            GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6;
            PWM0_3_CTL_R = 0;
            PWM0_3_GENA_R = (PWM_0_GENA_ACTCMPAD_M | PWM_0_GENA_ACTLOAD_ZERO);
            PWM0_3_CTL_R |= PWM_0_CTL_ENABLE;
            PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN;
            break;
        }
        break;

        case MODULE_1:

            SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
            SYSCTL_RCC_R &= ~(SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M);
            SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV ;
            SYSCTL_RCC_R |= ((Config_Ptr->clock)<<17);

            switch(Config_Ptr->gen_id)
            {
            case GEN_0: //PD0

                SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
                GPIO_PORTD_AFSEL_R |= (1<<0);
                GPIO_PORTD_AMSEL_R &= ~(1<<0);
                GPIO_PORTD_DEN_R |= (1<<0);
                GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD0_M);
                GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_M1PWM0;
                PWM1_0_CTL_R = 0;
                PWM1_0_GENA_R = (PWM_1_GENA_ACTCMPAD_M | PWM_1_GENA_ACTLOAD_ZERO);
                PWM1_0_CTL_R |= PWM_1_CTL_ENABLE;
                PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;
                break;

            case GEN_1: //PA6

                SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
                GPIO_PORTA_AFSEL_R |= (1<<6);
                GPIO_PORTA_AMSEL_R &= ~(1<<6);
                GPIO_PORTA_DEN_R |= (1<<6);
                GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA6_M);
                GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA6_M1PWM2;
                PWM1_1_CTL_R = 0;
                PWM1_1_GENA_R = (PWM_1_GENA_ACTCMPAD_M | PWM_1_GENA_ACTLOAD_ZERO);
                PWM1_1_CTL_R |= PWM_1_CTL_ENABLE;
                PWM1_ENABLE_R |= PWM_ENABLE_PWM2EN;
                break;

            case GEN_2: //PF0

                SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
                GPIO_PORTF_AFSEL_R |= (1<<0);
                GPIO_PORTF_AMSEL_R &= ~(1<<0);
                GPIO_PORTF_DEN_R |= (1<<0);
                GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF0_M);
                GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_M1PWM4;
                PWM1_2_CTL_R = 0;
                PWM1_2_GENA_R = (PWM_1_GENA_ACTCMPAD_M | PWM_1_GENA_ACTLOAD_ZERO);
                PWM1_2_CTL_R |= PWM_1_CTL_ENABLE;
                PWM1_ENABLE_R |= PWM_ENABLE_PWM4EN;
                break;

            case GEN_3: //PF2

                SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
                GPIO_PORTF_AFSEL_R |= (1<<2);
                GPIO_PORTF_AMSEL_R &= ~(1<<2);
                GPIO_PORTF_DEN_R |= (1<<2);
                GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF2_M);
                GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;
                PWM1_3_CTL_R = 0;
                PWM1_3_GENA_R = (PWM_1_GENA_ACTCMPAD_M | PWM_1_GENA_ACTLOAD_ZERO);
                PWM1_3_CTL_R |= PWM_1_CTL_ENABLE;
                PWM1_ENABLE_R |= PWM_ENABLE_PWM6EN;
                break;
            }
            break;
    }
}

/**************************************************************************************************
 * [Function Name]:   PWM_setValues
 *
 * [Description]:     Function to set load and compare value of PWM,
 * and pass generated signal through the chosen pin.
 *
 * [Arguments]:       Dynamic Configuration
 *
 * [Returns]:         void
 **************************************************************************************************/

void PWM_setValues (const PWM_Set * Set_Ptr)
{
    /*
     * ex:
     * PWM Module Clock Period = 1 / 16MHz = 62.5us
     * PWM Signal Period = 1 / 50Hz = 20ms
     * Load register value = 20ms/62.5us = 320000
     * Load register value for 50Hz square = 320000
     * Compare register value = (100% - 50%) of 320000 = 50% of 320000 = 160000
     */
    switch(Set_Ptr->module_id)
    {
    case MODULE_0:
        switch(Set_Ptr->gen_id)
        {
        case GEN_0:

            /* Load value : req_freq = f_cpu / load_value */
            PWM0_0_LOAD_R = (Set_Ptr->load_value);

            /* Compare value = (( period * percentage of duty cycle ) - 1) */
            PWM0_0_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
            break;

        case GEN_1:

            PWM0_1_LOAD_R = (Set_Ptr->load_value);
            PWM0_1_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
            break;

        case GEN_2:

            PWM0_2_LOAD_R = (Set_Ptr->load_value);
            PWM0_2_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
            break;

        case GEN_3:

            PWM0_3_LOAD_R = (Set_Ptr->load_value);
            PWM0_3_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
            break;
        }
        break;

        case MODULE_1:
            switch(Set_Ptr->gen_id)
            {
            case GEN_0:

                /* Load value : req_freq = f_cpu / load_value */
                PWM1_0_LOAD_R = (Set_Ptr->load_value);

                /* Compare value = (( period * percentage of duty cycle ) - 1) */
                PWM1_0_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
                break;

            case GEN_1:

                PWM1_1_LOAD_R = (Set_Ptr->load_value);
                PWM1_1_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
                break;

            case GEN_2:

                PWM1_2_LOAD_R = (Set_Ptr->load_value);
                PWM1_2_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
                break;

            case GEN_3:

                PWM1_3_LOAD_R = (Set_Ptr->load_value);
                PWM1_3_CMPA_R = ((( (Set_Ptr->duty_cycle) * (Set_Ptr->load_value) ) /100) - 1);
                break;

            }
            break;
    }
}

/**************************************************************************************************/
