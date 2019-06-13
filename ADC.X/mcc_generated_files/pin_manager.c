/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.35.9
        Device            :  PIC16F1788
        Driver Version    :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

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

#include <xc.h>
#include "pin_manager.h"
#include "stdbool.h"
#include "mcc.h"


void PIN_MANAGER_Initialize(void)
{
    /**
    LATx registers
    */   
    LATA = 0x00;    
    LATB = 0x00;    
    LATC = 0x00;    

    /**
    TRISx registers
    */    
    TRISE = 0x08;
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x40;

    /**
    ANSELx registers
    */   
    ANSELC = 0xBF;
    ANSELB = 0x7F;
    ANSELA = 0xBF;

    /**
    WPUx registers
    */ 
    WPUE = 0x00;
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x00;
    OPTION_REGbits.nWPUEN = 1;

    
    /**
    APFCONx registers
    */
    APFCON1 = 0x00;
    APFCON2 = 0x00;

    /////////////////////////////////////////////////////////////
    /*CONFIGURACION MANUAL*/
    /////////////////////////////////////////////////////////////
          
       
    WPUA =0;
    WPUB =0;
    WPUC =0;
    
    OPTION_REGbits.nWPUEN=0;
    
    
    TRISA = 0;      //Configuro pin como salida
    TRISB = 0;
    TRISC = 0;
    
    TRIS_TX = 0;             
    TRIS_RX = 1;        
    
    ANSELA = 0b00000000; 
    ANSELB = 0b00000000; 
    ANSELC = 0b00000000; 
    
    /*
    0: AOUT6 - AN4  - RA5
    1: AOUT5 - AN3  - RA3 
    2: AOUT4 - AN2  - RA2
    3: AOUT3 - AN1  - RA1
    4: AOUT9 - AN12 - RB0
    5: AOUT8 - AN10 - RB1
    6: AOUT7 - AN8  - RB2
    7: AOUT2 - AN9  - RB3
    8: AOUT1 - AN11 - RB4
    */
    
    TRISBbits.TRISB0 = 1;   //AN12  - OUT9
    TRISBbits.TRISB1 = 1;   //AN10  - OUT8
    TRISBbits.TRISB2 = 1;   //AN8   - OUT7
    TRISBbits.TRISB3 = 1;   //AN9   - OUT2
    TRISBbits.TRISB4 = 1;   //AN11  - OUT1

    TRISAbits.TRISA5 = 1;   //AN4  - OUT6
    TRISAbits.TRISA3 = 1;   //AN3  - OUT5
    TRISAbits.TRISA2 = 1;   //AN2  - OUT4
    TRISAbits.TRISA1 = 1;   //AN1  - OUT3
    
    
    ANSELBbits.ANSB0 = 1; //AN12  - OUT9
    ANSELBbits.ANSB1 = 1; //AN10  - OUT8
    ANSELBbits.ANSB2 = 1; //AN8   - OUT7
    ANSELBbits.ANSB3 = 1; //AN9   - OUT2
    ANSELCbits.ANSC5 = 1; //AN11  - OUT1
    
    
    ANSELAbits.ANSA5 = 1;   //AN4   - OUT6
    ANSELAbits.ANSA3 = 1;   //AN3   - OUT5
    ANSELAbits.ANSA2 = 1;   //AN2   - OUT4
    ANSELAbits.ANSA1 = 1;   //AN1   - OUT3

    
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;

    INTCON = 0x00; //Interrupts Control Register
    //PIE1=;			//Peripheral Interrupts
    //PIE2=;			//Peripheral Interrupts
    //PIR1=;			//Peripheral Interrupts
    //PIR2=;			//Peripheral Interrupts
 
    PCON = 0x00;

    //Watchdog 
    CLRWDT();
    WDTCON=0B00010110;	//b0: ENABLE b1..4:prescale 2^n ciclos 31kHz (LFINTOSC)	//Configuro WatchDog pero no lo habilito

    //TMR1 (16 bits)
    TMR1H = (INT_TIMER_1 >> 8); //Count Register (high)
    TMR1L = (INT_TIMER_1 & 0xFF); //Count Register (low)
    T1CON = 0B00000000; //Control Register
    TMR1IE = 1;
    T1CONbits.TMR1ON = 1;

    //ADC
    ADCON1bits.ADFM = 0;        //Signo magnitud
    ADCON1bits.ADCS = 0;
    ADCON1bits.ADPREF = 0;      //Referencia negativa masa
    ADCON2bits.TRIGSEL = 0;
    
    
    //Configuracion gral ADC
    FVRCONbits.ADFVR = 0b10;
    FVRCONbits.CDAFVR = 0;
    FVRCONbits.TSEN = 0;
    VREGCONbits.VREGPM=1;
    //ADC
    ADCON1bits.ADFM = 0;
    ADCON1bits.ADCS = 0;
    ADCON1bits.ADNREF = 0;
    ADCON1bits.ADPREF = 0;

    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.TMR1IE = 1;
   
    
}       

void PIN_MANAGER_IOC(void)
{   

}

/**
 End of File
*/