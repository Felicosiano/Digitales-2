
#include <xc.h>
#include "I2C_LCD.h"

unsigned char RS, i2c_add, BackLight_State = LCD_BACKLIGHT;

//---------------[ I2C Routines ]-------------------
//--------------------------------------------------
void I2C_Master_Init()
{
  SSP1ADD = 0x19;
  SSP1CON1  = 0x28;
  SSP1CON2 = 0x00;
  SSP1STAT = 0x00;

  TRISB6 = 1;
  TRISB4 = 1;

//Digital Inputs
  ANSELBbits.ANSB6 = 0;
  ANSELBbits.ANSB4 = 0;

//Assign PPS for Outputs
  RB4PPS = 0x08;      //SDA
  RB6PPS = 0x07;      //SCL

  RB4I2C = 0b00000001;    //Standard GPIO, I2C thresholds
  RB6I2C = 0b00000001;    //Standard GPIO, I2C Thresholds

//Assign PPS for Inputs
  SSP1DATPPS = 0b001100; // RB4
  SSP1CLKPPS = 0b001110; // RB6
 // clear the interrupt flags
  PIR1bits.SSP1IF = 0;
  PIR1bits.BCL1IF = 0;
  SSP1CON1bits.SSPEN = 1;
  
}

void I2C_Master_Wait()
{
    while ((SSP1STAT & 0x04) || (SSP1CON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SSP1CON2bits.SEN = 1;
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    SSP1CON2bits.RSEN = 1;
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    SSP1CON2bits.PEN = 1;
}

void I2C_ACK(void)
{
	SSP1CON2bits.ACKDT = 0;			// 0 -> ACK
	I2C_Master_Wait();
    SSP1CON2bits.ACKEN = 1;			// Send ACK
}

void I2C_NACK(void)
{
	SSP1CON2bits.ACKDT = 1;			// 1 -> NACK
    I2C_Master_Wait();
	SSP1CON2bits.ACKEN = 1;			// Send NACK
}

unsigned char I2C_Master_Write(unsigned char data)
{
    I2C_Master_Wait();
    PIR1bits.SSP1IF = 0;
    PIR1bits.BCL1IF = 0;
    SSP1BUF = data;
    
    while(!PIR1bits.SSP1IF);  // Wait Until Completion
	PIR1bits.SSP1IF = 0;
    return SSP1CON2bits.ACKSTAT;
}

unsigned char I2C_Read_Byte(void)
{
    //---[ Receive & Return A Byte ]---
	I2C_Master_Wait();
    SSP1CON2bits.RCEN = 1;		  // Enable & Start Reception
	while(!PIR1bits.SSP1IF);	  // Wait Until Completion
	PIR1bits.SSP1IF = 0;		  // Clear The Interrupt Flag Bit
    I2C_Master_Wait();
    return SSP1BUF;	  // Return The Received Byte
}
//======================================================

//---------------[ LCD Routines ]----------------
//------------------------------------------------------

void LCD_Init(unsigned char I2C_Add) 
{
  i2c_add = I2C_Add;
  
  I2C_Master_Start();
  IO_Expander_Write(0x00);
  I2C_Master_Stop();
  __delay_ms(30);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(LCD_RETURN_HOME);
  __delay_ms(5);
  LCD_CMD(0x20 | (LCD_TYPE << 2));
  __delay_ms(50);
  LCD_CMD(LCD_TURN_ON);
  __delay_ms(50);
  LCD_CMD(LCD_CLEAR);
  __delay_ms(50);
  LCD_CMD(LCD_ENTRY_MODE_SET | LCD_RETURN_HOME);
  __delay_ms(50);
}

void IO_Expander_Write(unsigned char Data) 
{
  I2C_Master_Start();
  I2C_Master_Write(i2c_add);
  I2C_Master_Write(Data | BackLight_State);
  I2C_Master_Stop();
}

void LCD_Write_4Bit(unsigned char Nibble) 
{
  // Get The RS Value To LSB OF Data  
  Nibble |= RS;
  IO_Expander_Write(Nibble | 0x04);
  IO_Expander_Write(Nibble & 0xFB);
  __delay_us(50);
}

void LCD_CMD(unsigned char CMD) 
{
  RS = 0; // Command Register Select
  LCD_Write_4Bit(CMD & 0xF0);
  LCD_Write_4Bit((CMD << 4) & 0xF0);
}

void LCD_Write_Char(char Data)
{
  RS = 1;  // Data Register Select
  LCD_Write_4Bit(Data & 0xF0);
  LCD_Write_4Bit((Data << 4) & 0xF0);
}

void LCD_Write_String(char* Str)
{
    for(int i=0; Str[i]!='\0'; i++)
       LCD_Write_Char(Str[i]); 
}

void LCD_Set_Cursor(unsigned char ROW, unsigned char COL) 
{    
  switch(ROW) 
  {
    case 2:
      LCD_CMD(0xC0 + COL-1);
      break;
    case 3:
      LCD_CMD(0x94 + COL-1);
      break;
    case 4:
      LCD_CMD(0xD4 + COL-1);
      break;
    // Case 1  
    default:
      LCD_CMD(0x80 + COL-1);
  }
}

void Backlight() 
{
  BackLight_State = LCD_BACKLIGHT;
  IO_Expander_Write(0);
}

void noBacklight() 
{
  BackLight_State = LCD_NOBACKLIGHT;
  IO_Expander_Write(0);
}

void LCD_SL()
{
  LCD_CMD(0x18);
  __delay_us(40);
}

void LCD_SR()
{
  LCD_CMD(0x1C);
  __delay_us(40);
}

void LCD_Clear()
{
  LCD_CMD(0x01); 
  __delay_us(40);
}