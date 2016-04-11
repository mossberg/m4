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

// Ultrasound

//void usound_init(void)
//{
//    TRISC3 = 1; // set pin as input
//    ANSC3 = 1; // select pin as analog in
//    ADCON0 = 0b00011101;
//    ADCON1 = 0b11110000;
//    ADCON2 = 0b00000000;
//}

//int16_t usound_read_old(void)
//{
//    ADCON0bits.GO_nDONE = 1;
//    while (ADCON0bits.GO);
//    uint16_t sensor_value = ((ADRESH << 8) + ADRESL);
//    float voltage = sensor_value * (3.3 / 1023.0);
//    float range = voltage / .0064;
//    return range;
//}

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


// Lidar

adc_result_t lidar_read(void)
{
    return ADC_GetConversion(LIDAR_ADC_CHAN);
}

//int32_t lidar_read_old(void)
//{
//    I2C_MESSAGE_STATUS stat;
//    uint8_t pdata[2] = {0, 4};
//    uint8_t upper = 0x0f;
//    uint8_t lower = 0x10;
//    uint8_t upperread, lowerread;
//
//    // TODO: this could fail, so put a while loop around it
//    I2C_MasterWrite(pdata, 2, 0x62, &stat);
//    while (stat == I2C_MESSAGE_PENDING);
//    if (stat != I2C_MESSAGE_COMPLETE)
//        return -1;
//
//    // this delay is *required*. per the lidar datasheet, as an alternative
//    // polling the device during reads, we can wait about 20ms and the
//    // measurement will be ready
//    __delay_ms(20);
//
//    stat = I2C_MESSAGE_PENDING;
//    
//    // read high 8 bits of measurement
//    I2C_MasterWrite(&upper, 1, 0x62, &stat);
//    while(stat == I2C_MESSAGE_PENDING);
//    I2C_MasterRead(&upperread, 1, 0x62, &stat);
//    while(stat == I2C_MESSAGE_PENDING);
//    if (stat != I2C_MESSAGE_COMPLETE)
//        return -1;
//
//    __delay_ms(1);
//    
//    // read low 8 bits of measurement
//    I2C_MasterWrite(&lower, 1, 0x62, &stat);
//    while(stat == I2C_MESSAGE_PENDING);
//    I2C_MasterRead(&lowerread, 1, 0x62, &stat);
//    while(stat == I2C_MESSAGE_PENDING);
//    if (stat != I2C_MESSAGE_COMPLETE)
//        return -1;
//    
//    __delay_ms(1);
//    return (upperread << 8) + lowerread;
//}

adc_result_t lidar_trigger(adc_result_t measurement)
{
    return measurement > LIDAR_THRESH;
}

// Feedback

void led_on(void)
{
    LED_PIN = 1;
}

void led_off(void)
{
    LED_PIN = 0;
}


void feedback_trigger(int trigger)
{
    if (trigger) {
        led_on();
    } else {
        led_off();
    }
}

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    //usound_init(); 

    // When using interrupts, you need to set the 
    // Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    TRISC0 = 0;
    
    // if there's ever an error in this loop, we ignore it and restart
    // the control loop
    while (1)
    {
        //uint16_t usound_dist = usound_read();
        adc_result_t lidar_dist = lidar_read();
//
        //feedback_trigger(lidar_trigger(lidar_dist) || usound_trigger(usound_dist));
        feedback_trigger(lidar_trigger(lidar_dist));
        //feedback_trigger(usound_trigger(usound_dist));
        
//        LED_PIN = 1;
//        __delay_ms(100);
//        LED_PIN = 0;
//        __delay_ms(100);
    }
}

#if 0
void pwm_output_disable(void) {
    PWM1CON &= ~(1<<6);
    PWM3CON &= ~(1<<6);
}

void pwm_output_enable(void) {
    PWM1CON |= (1<<6);
    PWM3CON |= (1<<6);
}

/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1)
    {
        // Add your application code
        pwm_output_enable();
        __delay_ms(1000);
        pwm_output_disable();
        __delay_ms(1000);
    }
}
/**
 End of File
*/
#endif
