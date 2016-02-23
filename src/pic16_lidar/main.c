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

/*
 * I2C Primitives
 */

/*
Function: I2CInit
Return:
Arguments:
Description: Initialize I2C in master mode, Sets the required baudrate
*/
void I2CInit(void)
{
	TRISC0 = 1;      /* SDA and SCL as input pin */
	TRISC1 = 1;      /* these pins can be configured either i/p or o/p */
    /* maybe replace this with an assignment */
	SSP1STAT |= 0x80; /* Slew rate disabled */
	SSP1CON1 = 0x28;   /* SSPEN = 1, I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) */
	SSP1ADD = 0x28;    /* 100Khz @ 4Mhz Fosc */
}
 
/*
Function: I2CStart
Return:
Arguments:
Description: Send a start condition on I2C Bus
*/
void I2CStart()
{
	SEN = 1;         /* Start condition enabled */
	while(SEN);      /* automatically cleared by hardware */
                     /* wait for start condition to finish */
}
 
/*
Function: I2CStop
Return:
Arguments:
Description: Send a stop condition on I2C Bus
*/
void I2CStop()
{
	PEN = 1;         /* Stop condition enabled */
	while(PEN);      /* Wait for stop condition to finish */
                     /* PEN automatically cleared by hardware */
}
 
/*
Function: I2CRestart
Return:
Arguments:
Description: Sends a repeated start condition on I2C Bus
*/
void I2CRestart()
{
	RSEN = 1;        /* Repeated start enabled */
	while(RSEN);     /* wait for condition to finish */
}
 
/*
Function: I2CAck
Return:
Arguments:
Description: Generates acknowledge for a transfer
*/
void I2CAck()
{
	ACKDT = 0;       /* Acknowledge data bit, 0 = ACK */
	ACKEN = 1;       /* Ack data enabled */
	while(ACKEN);    /* wait for ack data to send on bus */
}
 
/*
Function: I2CNck
Return:
Arguments:
Description: Generates Not-acknowledge for a transfer
*/
void I2CNak()
{
	ACKDT = 1;       /* Acknowledge data bit, 1 = NAK */
	ACKEN = 1;       /* Ack data enabled */
	while(ACKEN);    /* wait for ack data to send on bus */
}
 
/*
Function: I2CWait
Return:
Arguments:
Description: wait for transfer to finish
*/
void I2CWait()
{
	while ((SSPCON2 & 0x1F ) || ( SSPSTAT & 0x04 ) );
    /* wait for any pending transfer */
}
 
/*
Function: I2CSend
Return:
Arguments: dat - 8-bit data to be sent on bus
           data can be either address/data byte
Description: Send 8-bit data on I2C bus
*/
void I2CSend(unsigned char dat)
{
	SSPBUF = dat;    /* Move data to SSPBUF */
	while(BF);       /* wait till complete data is sent from buffer */
	I2CWait();       /* wait for any pending transfer */
}
 
/*
Function: I2CRead
Return:    8-bit data read from I2C bus
Arguments:
Description: read 8-bit data from I2C bus
*/
unsigned char I2CRead(void)
{
	unsigned char temp;
/* Reception works if transfer is initiated in read mode */
	RCEN = 1;        /* Enable data reception */
	while(!BF);      /* wait for buffer full */
	temp = SSPBUF;   /* Read serial buffer and store in temp register */
	I2CWait();       /* wait to check any pending transfer */
	return temp;     /* Return the read data from bus */
}

/*
 * I2C Abstraction Layer
 * modeled off of arduino
 */

/* uint8_t i2c_write(uint8_t device_addr, uint8_t reg_addr, uint8_t data) */
/* { */
/*     uint8_t ret = 0; */

/*     // trigger start condition on bus */
/*     I2CStart(); */
    
/* } */




/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();
TRISC2 = 0;
    I2CInit();
    
    while(1)
    {
        I2CStart();
          I2CSend(0x55); 
          I2CStop();
          
          RC2 = 1;
        __delay_ms(100);  // 1 second delay 
        RC2 = 0;
        __delay_ms(100);  // 1 second delay 
        
        //__delay_ms(500);  // 1 second delay 
   // RC0=0;                    // make RD7 pin Low to Off LED 
   // __delay_ms(500);
        /* TODO <INSERT USER APPLICATION CODE HERE> */
    }

}

