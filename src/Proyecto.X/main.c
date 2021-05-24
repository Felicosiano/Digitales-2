/*
 * File:   main.c
 * Author: Khaled Magdy
 * LAB:    34 - I2C LCD Interfacing
 */

#include <xc.h>
#include "config.h"
#include "I2C_LCD.h"

void main(void) {
    
    I2C_Master_Init();
    LCD_Init(0x4E);    // Initialize LCD module with I2C address = 0x4E
 
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("  Khaled Magdy");
    LCD_Set_Cursor(2, 1);  
    LCD_Write_String("    DeepBlue");
    __delay_ms(2500);

    while(1)
    {
       LCD_SR();
       __delay_ms(350); 
       LCD_SR();
       __delay_ms(350); 
       LCD_SL();
       __delay_ms(350);
       LCD_SL();
       __delay_ms(350);
    }
    return;
}
