/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC16F1503
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#include "mcc_generated_files/mcc.h"

#define LIDAR_THRESH 500
#define USOUND_THRESH 50

#define LED_PIN RC0

#define USOUND_ADC_CHAN 7
#define LIDAR_ADC_CHAN 6


void error(void) {
    LED_PIN = 1;
    while(1);
}

void led_on(void)
{
    LED_PIN = 1;
}

void led_off(void)
{
    LED_PIN = 0;
}

void pwm1_output_disable(void) {
    for(int i = 0; i < 15; i++)
        PWM1CON &= ~(1<<6);
    //PWM1CONbits.PWM1OE = 0;
    
}

void pwm1_output_enable(void) {
//    PWM1CON |= (1<<6);
    PWM1CONbits.PWM1OE = 1;
}

void pwm3_output_disable(void) {
//    PWM3CON &= ~(1<<6);
    PWM3CONbits.PWM3OE = 0;
}

void pwm3_output_enable(void) {
//    PWM3CON |= (1<<6);
    PWM3CONbits.PWM3OE = 1;
}

// Ultrasound

uint16_t usound_read(void)
{
    adc_result_t sensor_value = ADC_GetConversion(USOUND_ADC_CHAN);
    float voltage = sensor_value * (3.3 / 1023.0);
    float range = voltage / .0064;
    return (uint16_t) range;
}

uint32_t usound_trigger(adc_result_t measurement)
{
    return measurement < USOUND_THRESH;
}

void usound_feedback(adc_result_t trigger)
{
    if (usound_trigger(trigger)) {
        pwm3_output_enable();
        led_on();
    } else {
        pwm3_output_disable();
        led_off();
    }
}


// Lidar

adc_result_t lidar_read(void)
{
    return ADC_GetConversion(LIDAR_ADC_CHAN);
}

adc_result_t lidar_trigger(adc_result_t measurement)
{
    return measurement > LIDAR_THRESH;
}

void lidar_feedback(adc_result_t trigger)
{
    if (lidar_trigger(trigger)) {
        pwm1_output_enable();
        led_on();
    } else {
        pwm1_output_disable();
        led_off();
    }
}

// Feedback

void feedback_trigger(int trigger)
{
    if (trigger) {
        led_on();
    } else {
        led_off();
    }
}



// Main

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    TRISC0 = 0;
    
    while (1)
    {
        uint16_t usound_dist = usound_read();
        adc_result_t lidar_dist = lidar_read();

        lidar_feedback(lidar_dist);
        usound_feedback(usound_dist);
        //feedback_trigger(lidar_trigger(lidar_dist) || usound_trigger(usound_dist));
        //feedback_trigger(lidar_trigger(lidar_dist));
        //feedback_trigger(usound_trigger(usound_dist));
        
//        LED_PIN = 1;

//        pwm1_output_enable();
//        __delay_ms(1000);
////        LED_PIN = 0;
//        pwm1_output_disable();
//        __delay_ms(1000);
//        pwm3_output_enable();
//        __delay_ms(1000);
////        LED_PIN = 0;
//        pwm3_output_disable();
//        __delay_ms(1000);
    }
}
