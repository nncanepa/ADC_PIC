/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC16F1788
        Driver Version    :  2.00
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"

union Data DatosADC;
uint16_t RealizarMedicion;
uint16_t EnviarDatos;
/* Conexion final de los pines de configuracion -> despues del recableado
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
//CHS:      AN4 - AN3 - AN2 - AN1 
//          AN12 - AN10 - AN8 - AN9 - AN11
uint8_t OrdenConversionADC[] = {0b00100, 0b00011, 0b00010, 0b00001,0b01100, 
                                0b01010, 0b01000, 0b01001, 0b01011};


/*Main application*/
void main(void)
{
    uint8_t i;
    char j;
    // initialize the device
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    while (1)
    {
        MedirADCs();            //Se ejecuta cada 5mseg
        EnviarDatos2ESP();      //Se ejecuta cada 100mseg
    }

}



void EnviarDatos2ESP(void)
{
    char i;
    uint16_t checksum;
    checksum=0;
    
    if(EnviarDatos)        //Si no paso el tiempo vuelve
        return;
    
    EnviarDatos = 100;       //Recargo periodo de muestreo (5mseg)
    
    
    for(i = 0; i < CANALES_ADC; i++){
       checksum ^= DatosADC.LecturasADC[i];
    }
    
    DatosADC.LecturasADC[CANALES_ADC] = checksum;
   
    printf("LECTURAS: ");
    for(i=0;i<SIZOF_DATA_ESP;i++)
    {
        EUSART_Write(DatosADC.str[i]);
    }
}

/*
 ADC: 12bits - Referencia de 2.048V  
 */

void MedirADCs(void)
{
    uint8_t i;
    int16_t MedicionesActuales[9];
    
    if(RealizarMedicion)        //Si no paso el tiempo vuelve
        return;
    
    RealizarMedicion = 5;       //Recargo periodo de muestreo (5mseg)

    FVRCONbits.ADFVR = 0b10;    //Configuracion de la refencia del ADC 2.048V
    FVRCONbits.CDAFVR = 0;
    FVRCONbits.TSEN = 0;
    FVRCONbits.FVREN = 1;
    __delay_ms(10);
    while(!FVRCONbits.FVRRDY);

    //ADC
    ADCON0bits.ADRMD = 0; //12bits
    //ADCON2bits.CHSN = 0b111;  //Mati te falto un 1 LPQTP
    ADCON2bits.CHSN = 0b1111;  //Referencia negativa del ADC al registro ADNREF
    ADCON1bits.ADFM = 1;
    ADCON1bits.ADCS = 0b110;      //Fosc/64 Mas rapido no se puede 
    ADCON1bits.ADNREF = 0;
    ADCON1bits.ADPREF = 0b11;     //Referencia interna de 2.048V
    ADCON0bits.ADRMD = 0;

    
    for(i=0; i< CANALES_ADC; i++)
    {
        ADCON0bits.CHS = OrdenConversionADC[i];
        __delay_us(20);     //Le doy tiempo al cambio de canal y carga cap.
        ADON = 1; //ACTIVO ADC
        ADCON0bits.GO = 1;
        while(ADCON0bits.GO_nDONE);     //Espera muestreo
        MedicionesActuales[i] = (((uint16_t)ADRESH)<<8) | ADRESL;
    
        DatosADC.LecturasADC[i] = DatosADC.LecturasADC[i] + ((MedicionesActuales[i] - DatosADC.LecturasADC[i])/16);
        
        if(DatosADC.LecturasADC[i] < 0) {
            DatosADC.LecturasADC[i] = 0;
        }
        
    }
}