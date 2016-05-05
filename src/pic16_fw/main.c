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

#define NPTS 12

#define LIDAR_DIST_THRESH 900
#define LIDAR_DERIV_THRESH 500
#define USOUND_THRESH 50

#define LED_PIN RC0

#define USOUND_ADC_CHAN 7
#define LIDAR_ADC_CHAN 6

#define PWM_OFF 0
#define PWM_50 510
#define PWM_100 1021



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
    PWM1_LoadDutyValue(PWM_OFF);
}

void pwm1_output_enable(void) {
    PWM1_LoadDutyValue(PWM_50);
    PWM1CONbits.PWM1OE = 1;
}

void pwm1_strong_pulse_100(void) {
    PWM1_LoadDutyValue(PWM_100);
    PWM1CONbits.PWM1OE = 1;
    __delay_ms(100);
    pwm1_output_disable();
}

void pwm1_strong_pulse_2000(void) {
    PWM1_LoadDutyValue(PWM_100);
    PWM1CONbits.PWM1OE = 1;
    __delay_ms(2000);
    pwm1_output_disable();
}

void pwm3_output_disable(void) {
    PWM3_LoadDutyValue(PWM_OFF);
}

void pwm3_output_enable(void) {
    PWM3_LoadDutyValue(PWM_50);
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
    //return;
    if (usound_trigger(trigger)) {
        pwm3_output_enable();
        //led_on();
    } else {
        pwm3_output_disable();
        //led_off();
    }
}


// Lidar

adc_result_t lidar_read(void)
{
    return ADC_GetConversion(LIDAR_ADC_CHAN);
}

adc_result_t lidar_depth_detect(adc_result_t measurement)
{
    return measurement > LIDAR_DIST_THRESH;
}

void lidar_feedback(adc_result_t trigger)
{
    if (lidar_depth_detect(trigger)) {
        pwm1_output_enable();
        //led_on();
    } else {
        pwm1_output_disable();
        //led_off();
    }
}

// Edge Detection
// returns boolean based on whether any of the
// computed derivatives exceeds a threshold
// arg: lidar reading
uint8_t lidar_edge_detect(int16_t lidar_dist)
{
    static uint8_t startup_fill_index = 0;
    
    // temporary output values and sensor value
    int16_t vtmp[3];

    // variables to compute the standard deviation
    // int16_t x = 0, x2 = 0;

    // input and output buffer
    static int16_t idata[NPTS];
    int16_t odata[NPTS];
    uint8_t i, j;

    if (startup_fill_index < NPTS){
        idata[startup_fill_index] = lidar_dist;
        startup_fill_index++;
        return 0;
    }
    else{
        for(i = 0; i < NPTS-1; i++)
        {
            idata[i] = idata[i+1];
        }
        //__delay_ms(50); --> moved to main
        idata[NPTS-1] = lidar_dist;
    }

    // apply filter, compute the standard deviation
    for(i = 0; i < NPTS/3; i++)
    {
        // apply the filter to the idata
        vtmp[0] =   -idata[0+i*3] +   idata[2+i*3];
        vtmp[1] = -2*idata[0+i*3] + 2*idata[2+i*3];
        vtmp[2] =   -idata[0+i*3] +   idata[2+i*3];

        // write the filtered results to odata
        odata[0+i*3] = vtmp[0];
        odata[1+i*3] = vtmp[1];
        odata[2+i*3] = vtmp[2];

    }

    // check threshold
    for(i = 0; i < NPTS; i++)
    {
        // check the raw threshold on the output of the filter
        if(odata[i] > LIDAR_DERIV_THRESH)
        {
            startup_fill_index = 0;          
            return 1;
        }
    }
    return 0;
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
    // 5000/32
    __delay_ms(156);
    // initialize the device
    SYSTEM_Initialize();

    TRISC0 = 0;
    uint8_t is_over_edge = 0;
    while (1)
    {
        __delay_ms(2);//max read time for lidar module
       
        uint16_t usound_dist = usound_read();
        // overhead objects
        usound_feedback(usound_dist);  
        
        adc_result_t lidar_dist = lidar_read();
        uint16_t is_edge_found = lidar_edge_detect(lidar_dist);
        
        if (is_edge_found) {
            pwm1_strong_pulse_100();
            is_over_edge = lidar_depth_detect(lidar_read());
        }
        
        if (is_over_edge) {
            pwm1_output_enable();
            //__delay_ms(5000);
            is_over_edge = lidar_depth_detect(lidar_dist);
        } else {
            pwm1_output_disable();
        }
    }
}
