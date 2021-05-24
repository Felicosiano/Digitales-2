/*
/////////////////// DATOS DEL PROGRAMA ////////////////////
//  TTITULO: Autonomous irrigation system for medium crops
//  MICRO:PIC16F15244
//  ESTUDIANTES: Parra B. Jose L. 2420181011
//               Cortes L. Willy A. 2420151007
//               Enciso N. David F. 2420171016
//  Profesor: Harold F MURCIA
//  FECHA: 24 de mayo de 2021
////////////////////////////////////////////////////////// */
#pragma config FEXTOSC = OFF    // External Oscillator Mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_32MHZ// Power-up Default Value for COSC bits (HFINTOSC (32 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O function on RA4)
#pragma config VDDAR = HI       // VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 2.3V - 5.5V)

// CONFIG2
#pragma config MCLRE = INTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config WDTE = OFF     // WDT Operating Mode bits (WDT disabled; SEN is ignored)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config PPS1WAY = ON     // PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block is disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF is disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block is not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block is not write-protected)
#pragma config WRTC = OFF       // Configuration Registers Write Protection bit (Configuration Registers are not write-protected)
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit (SAF is not write-protected)
#pragma config LVP = OFF       // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.)

// CONFIG5
#pragma config CP = OFF         // User Program Flash Memory Code Protection bit (User Program Flash Memory code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#include <stdlib.h>
#include <stdio.h>
#include <pic16f15244.h>



#ifndef CLOCK_H
#define	CLOCK_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 32000000
#endif

void CLOCK_Initialize(void);

#endif	/* CLOCK_H */
/**
 End of File
*/
#include "I2C_LCD.h"
float lecturaUv,lecturaHumedad,lecturaTemp=0;
const unsigned char AUV=0b000010 ,ASH=0b000101 ,AST=0b000001,PIN_VAL=0b00000001,PIN_BOM=0b00000010,nivel=0b00001100;
char flag=0;

int * status;
// CTEMP es la constante de conversion voltios=>Temperatura y viene de V=T/100;
//CHUM constante de voltios a humedad
//CUV constante de conversion de voltios a radiacion uv

const float CTEMP=100.0*5.0/1032.0,CHUM=1.0*5.0/1032.0,CUV=7.894*5.0/1032.0;

// constantes limites para condicion de riego
const int MAXTEMP=26,MINHUM=2,MAXUV=8;

char* texuv;

char* texT;

char* texH;

void Iniciar_ADC()
{
    ADCON0 = 0b00000001; //
    ADCON1 = 0b11100000;
}
unsigned int LeerAdc(unsigned char canal)
{ 
    ADCON0 &=0X3;
    
    ADCON0 |=canal<<2;
    
    __delay_us(20);
    //GO_nDONE=1;

    ADCON0 |=1<<1;
    while(ADCON0 & 0x2);
    
    return ((ADRESH<<8)+ADRESL);
}
void PutVariables()
{
   
    texuv=ftoa(lecturaUv,status);//3.1415...
    texuv[4]=0x0;//3.141 
    LCD_Set_Cursor(1, 4);
    LCD_Write_String(texuv);
    
    texT=ftoa(lecturaTemp,status);
    texT[4]=0x0;
    LCD_Set_Cursor(1, 10); 
    LCD_Write_String(texT);  
    
    texH=ftoa(lecturaHumedad,status);
    texH[4]=0x0;
    LCD_Set_Cursor(2, 3); 
    LCD_Write_String(texH);
    
    

}





void PutTexto()
{
    LCD_Write_String("Uv=");
    LCD_Set_Cursor(1, 8); 
    LCD_Write_String("T=");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("H=");
}
void inicio()
{
    
    I2C_Master_Init();
    
    LCD_Init(0x4E);
    PutTexto();     
}
void LeerSensores()
{
    //lectura sensores digitales
    //CTEMP=100.0,CHUM=1.0,CUV=0.1266;
    //lectura analogicos 
    lecturaUv=CUV*((LeerAdc(AUV)))-1; // ecuacion de la intensidad uv = Vsensado*pendiente - vcorte
    lecturaHumedad=CHUM*(LeerAdc(ASH));
    lecturaTemp=CTEMP*(LeerAdc(AST));
    
}
void TanqueLLeno()
{
    while(flag==1)
    {
       
        if((PORTC & nivel)==0x0)
            
        {
            flag=0;
            PORTC|=PIN_VAL;// off Val
            LCD_Set_Cursor(2, 8);
            LCD_Write_String("TanqFull");
            __delay_ms(1000);
        }
        
        //else flag=1;
    }
}
void TanqueVacio()
{    
    PORTCbits.RC7=PORTCbits.RC2;
    PORTCbits.RC6=PORTCbits.RC3;
    if((PORTC & nivel)==0b00001100)
    {
        PORTC&=~PIN_VAL; // on Val
        
        flag=1;
        LCD_Set_Cursor(2, 8);
        LCD_Write_String("Llenando");
        PORTC|=PIN_BOM;// ooff BOM
    }
    
    
}
void CondicionesDeRiego()
{
    if( (lecturaTemp>=MAXTEMP) && (lecturaHumedad>=MINHUM) )
    {
        
        PORTC&=~PIN_BOM; // ON BOMBA
        LCD_Set_Cursor(2, 8);
        LCD_Write_String("Regando ");
        
    }
    else
    {
        PORTC|=PIN_BOM;
        LCD_Set_Cursor(2, 8);
        LCD_Write_String("Suelo Ok");       
    }
}

void main(void) {
    TRISC=nivel; // configurar pines I/O
    ANSELC=0x00;
    
    PORTC|=0b11110011;
    __delay_ms(800);
    PORTC&=~0b11110000;
    
    
    TRISB=0b01010000;
    TRISA=0b111111;
    ANSELA=0xff;
    //PutVariables(); 
    
    Iniciar_ADC();
    inicio();


    while(1)
    {

        LeerSensores();
        PutVariables();
        TanqueVacio();
        TanqueLLeno();
        CondicionesDeRiego();
        __delay_ms(500);
    }
    return;
}
