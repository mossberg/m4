/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */

#define _XTAL_FREQ 8000000
/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

/* i.e. uint8_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();
    TRISC0 = 0;
    
    TRISC1 = 1; // set pin as input
    ANSC1 = 1; // select pin as analog in
    ADCON0 = 0b00010101;
    ADCON1 = 0b11110000;
    ADCON2 = 0b00000000;
    //ADCS = 0b11;
    //CHS = 5;
    while(1)
    {
        
        ADCON0bits.GO_nDONE = 1;
        while (ADCON0bits.GO);
        int x = ((ADRESH * 256) + ADRESL);
        if (x > 50) {
            RC0 = 0;
        }else {
             RC0 = 1;
        }
        //__delay_ms(500);  // 1 second delay 
   // RC0=0;                    // make RD7 pin Low to Off LED 
   // __delay_ms(500);
        /* TODO <INSERT USER APPLICATION CODE HERE> */
    }

}

