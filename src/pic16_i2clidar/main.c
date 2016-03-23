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
#define FAST 20
#define SLOW 100
#define SUPERSLOW 2000
void flooop(void)
{
    while (1) {
        RC2 = 1;
        __delay_ms(FAST);  // 1 second delay 
        RC2=0;                    // make RD7 pin Low to Off LED 
        __delay_ms(FAST);
    }
}

void slooop(void)
{
    while (1) {
        RC2 = 1;
        __delay_ms(SLOW);  // 1 second delay 
        RC2=0;                    // make RD7 pin Low to Off LED 
        __delay_ms(SLOW);
    }
}

void sslooop(void)
{
    while (1) {
        RC2 = 1;
        __delay_ms(SUPERSLOW);  // 1 second delay 
        RC2=0;                    // make RD7 pin Low to Off LED 
        __delay_ms(SUPERSLOW);
    }
}



/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the 
    // Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    TRISC2 = 0;
    I2C_MESSAGE_STATUS stat;
    uint8_t pdata[2] = {0, 4};
    uint8_t a = 0;
    uint8_t b = 4;
    uint8_t tmp;
    //uint8_t length = sizeof(pdata)/sizeof(*pdata);
    while (1)
    {
        I2C_MasterWrite(pdata, 2, 0x62, &stat);
        //I2C_MasterWrite(&b, 1, 0x55, &stat);
        //I2C_MasterRead(&tmp, 1, 0x55, &stat);
        
        while(stat == I2C_MESSAGE_PENDING) {
            RC2 = 1;
            __delay_ms(29);  // 1 second delay 
            RC2=0;                    // make RD7 pin Low to Off LED 
            __delay_ms(29);
        }
        if (stat == I2C_DATA_NO_ACK) {
                while (1) {
                      RC2 = 1;
                       __delay_ms(100);  // 1 second delay 
                    RC2=0;                    // make RD7 pin Low to Off LED 
                        __delay_ms(100);
               }
        }
        sslooop();
        
        if (stat == I2C_MESSAGE_PENDING) {
        
        //if (stat == I2C_MESSAGE_FAIL) {
            sslooop();
        }
        RC2 = 1;
        
        __delay_ms(100);  // 1 second delay 
        RC2 = 0;                    // make RD7 pin Low to Off L            ED 
        __delay_ms(100);
        // Add your application code
        
    }
}
/**
 End of File
*/
