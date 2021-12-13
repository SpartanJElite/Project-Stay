/*
 * File:   IMU.c
 * Author: spong
 *
 * Created on December 10, 2021, 2:41 PM
 */


#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF
#define DEBUG 1
#define I2C_BaudRate 100000

unsigned char Status_temp = 0x00; //Status
unsigned char Wreg_temp = 0x00; //WREG
unsigned char BSR_temp = 0x00; //BSR

void Initial(void); //Initialize Function
void I2C_Master_Init(void);
void I2C_Master_Wait(void);
void I2C_Master_Start(void);
void I2C_Start(char add);
void I2C_Master_RepeatedStart(void);
void I2C_ACK(void);
void I2C_NACK(void);
unsigned char I2C_Master_Write(unsigned char data);
unsigned char I2C_Read_Byte(void);
unsigned char I2C_Read(unsigned char ACK_NACK);
void MPU6050(void);
void MPU6050_Read(void);

void UART_TX_Init(void);
void UART_Write(unsigned char data);
void UART_Write_String(char* buf);

void main(void) {
    Initial();
    while(1)
	{
	    MPU6050_Read();
        __delay_ms(50);
    }
    return;
}

void Initial(void){
    RCONbits.IPEN = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    IPR1bits.SSP1IP = 0;
    PIE1bits.SSP1IE = 1;
    PIR1bits.SSP1IF = 0;
    
    
    
    UART_TX_Init();
    MPU6050();
}

void I2C_Master_Init(void){
    SSP1CON1 = 0x28;
    SSP1CON2 = 0x00;
    SSP1STAT = 0x00;
    SSP1ADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
    TRISCbits.TRISC3 = 1; //SCL
    TRISCbits.TRISC4 = 1; //SDA
    return;
}

void I2C_Master_Wait(void){
    while((SSP1STAT & 0x04) || (SSP1CON2 & 0x1F)){
    }
}

void I2C_Master_Start(void){
    I2C_Master_Wait();
    SSP1CON2bits.SEN = 1;
}

void I2C_Start(char add){
    I2C_Master_Wait();
    SSP1CON2bits.SEN = 1;
    I2C_Master_Write(add);
}

void I2C_Master_RepeatedStart(void){
    I2C_Master_Wait();
    SSP1CON2bits.RSEN = 1;
}

void I2C_Master_Stop(){
    I2C_Master_Wait();
    SSP1CON2bits.PEN = 1;
}

void I2C_ACK(void){
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN == 1){
    }
}

void I2C_NACK(void){
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN == 1){
    }
}

unsigned char I2C_Master_Write(unsigned char data){
    I2C_Master_Wait();
    SSP1BUF = data;
    while(!PIR1bits.SSP1IF){
    }
    PIR1bits.SSP1IF = 0;
    return SSP1CON2bits.ACKSTAT;
}

unsigned char I2C_Read_Byte(void){
    I2C_Master_Wait();
    SSP1CON2bits.RCEN = 1;
    while(!PIR1bits.SSP1IF){
    }
    PIR1bits.SSP1IF = 0;
    I2C_Master_Wait();
    return SSP1BUF;
}

unsigned char I2C_Read(unsigned char ACK_NACK){
    unsigned char Data;
    SSP1CON2bits.RCEN = 1;
    while(!SSP1STATbits.BF){
    }
    Data = SSP1BUF;
    if(ACK_NACK == 0){
        I2C_ACK();
    }
    else{
        I2C_NACK();
    }
    while(!PIR1bits.SSP1IF){
    }
    PIR1bits.SSP1IF = 0;
    return Data;
}

void MPU6050(){
    __delay_ms(100);
    I2C_Master_Init();
    
    //Sample Rate
    I2C_Start(0xD0);
    I2C_Master_Write(0x19);
    I2C_Master_Write(0x07);
    I2C_Master_Stop();
    
    //Clock Source
    I2C_Start(0xD0);
    I2C_Master_Write(0x6B);
    I2C_Master_Write(0x01);
    I2C_Master_Stop();
    
    //DLPF
    I2C_Start(0xD0);
    I2C_Master_Write(0x1A);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();
    
    //ACCEL
    I2C_Start(0xD0);
    I2C_Master_Write(0x1C);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();
    
    //GYRO
    I2C_Start(0xD0);
    I2C_Master_Write(0x1B);
    I2C_Master_Write(0x18);
    I2C_Master_Stop();
    
    //Interrupt
    I2C_Start(0xD0);
    I2C_Master_Write(0x56);
    I2C_Master_Write(0x01);
    I2C_Master_Stop();
}

void MPU6050_Read(void){
    char buffer[40];
    int Ax,Ay,Az,Gx,Gy,Gz,T;
    float AX,AY,AZ,GX,GY,GZ,t;
    
    I2C_Start(0xD0);
    I2C_Master_Write(0x3B);
    I2C_Master_Stop();
    I2C_Start(0xD1);
    I2C_Read(0);
    Ax = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    Ay = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    Az = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    T  = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    Gx = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    Gy = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);
    Gz = ((int)I2C_Read(0)<<8) | (int)I2C_Read(1);
    I2C_Master_Stop();
    
    AX = (float)Ax/16384.0;
    AY = (float)Ay/16384.0;
    AZ = (float)Az/16384.0;
    GX = (float)Gx/131.0;
    GY = (float)Gy/131.0;
    GZ = (float)Gz/131.0;
    t = ((float)T/340.00)+36.53;
    
    // Print The Results
    sprintf(buffer,"Ax = %.2f \t",AX);	
    UART_Write_String(buffer);
   
    sprintf(buffer," Ay = %.2f \t",AY);
    UART_Write_String(buffer);
		
    sprintf(buffer," Az = %.2f \t",AZ);
    UART_Write_String(buffer);

    sprintf(buffer," T = %.2f  ",t);
    UART_Write_String(buffer);

    sprintf(buffer," Gx = %.2f \t",GX);
    UART_Write_String(buffer);

    sprintf(buffer," Gy = %.2f \t",GY);
    UART_Write_String(buffer);
   
    sprintf(buffer," Gz = %.2f\r\n",GZ);
    UART_Write_String(buffer);
}

void UART_TX_Init(void){
    TRISCbits.TRISC7 = 1;
    TRISCbits.TRISC6 = 0;
    BAUDCON1bits.BRG16 = 0;
    BAUDCON1bits.RCIDL = 1;
    SPBRG1 = 12;
    SPBRGH1 = 0x00;
    TXSTA1bits.BRGH = 0;
    TXSTA1bits.TX9 = 0;
    TXSTA1bits.TXEN = 1;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN = 1;
    
    IPR1bits.RC1IP = 0;
    PIE1bits.RC1IE = 1;
}

void UART_Write(unsigned char data){
    while(PIR1bits.TX1IF == 0){
        }
    TXREG1 = data;
}

void UART_Write_String(char* buf){
    int i = 0;
    while(buf[i] != '\0')
        UART_Write(buf[i++]);
}