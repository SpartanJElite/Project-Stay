/*
 * File:   Le_Jason_Project
 * Author: Jason Le
 *
 * Created on December 9, 2021, 9:10 PM
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
#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  (FROM ARDUINO LIB)
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo   (FROM ARDUINO LIB)
#define I2C_BaudRate 100000

unsigned char TMR1X = 0; //Extension Byte for TMR1
unsigned char CCPR1X = 0; //Extension Byte for CCP1
unsigned char TMR3X = 0; //Extension Byte for TMR3
unsigned char CCPR4X = 0; //Extension Byte for CCP4
unsigned char Status_temp = 0x00; //Status
unsigned char Wreg_temp = 0x00; //WREG
unsigned char BSR_temp = 0x00; //BSR
unsigned long Servo1_on = 4000; //Initialize PWM ON for Servo1
unsigned long Servo1_off = 76000; //Initialize PWM OFF for Servo1
unsigned long Servo2_on = 4000; //Initialize PWM ON for Servo2
unsigned long Servo2_off = 76000; //Initialize PWM OFF for Servo2
unsigned long time0counter = 0;
float CurrentTime = 0;
float previousTime = 0;
float GyroAngleX = 0;
float GyroAngleY = 0;
float yaw = 0;

void Initial(void); //Initialize Function

void CCP1handler(void); //CCP1 Function
void CCP4handler(void); //CCP4 Function
void TMR1handler(void); //TMR1 Function
void TMR3handler(void); //TMR3 Function
void Servo1(unsigned int angle); //Servo1 Write
void Servo2(unsigned int angle); //Servo2 Write

void I2C_Master_Wait(void);
void I2C_Master_Start(void);
void I2C_Start(char add);
void I2C_Master_RepeatedStart(void);
void I2C_Master_Stop(void);
void I2C_ACK(void);
void I2C_NACK(void);
unsigned char I2C_Master_Write(unsigned char data);
unsigned char I2C_Read_Byte(void);
unsigned char I2C_Read(unsigned char ACK_NACK);

void MPU6050_Read(void);
void TMR0handler(void);

void UART_TX_Init(void);
void UART_Write(unsigned char data);
void UART_Write_String(char* buf);



unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax); //map function(ARDUINO)
void main(void) {
    Initial();
    Servo1(45);
    Servo2(45);
    while(1){
        for( int i = 180;i >= 0;i --){
//            Servo1(i);
//            Servo2(i);
            MPU6050_Read();
            __delay_ms(50);
        }
        //MPU6050_Read();  
    }
    return;
}

void Initial(void){
    //PORT CONFIGURE
    TRISC = 0x00;
    LATC = 0x00;
    TRISD = 0x00;
    LATD = 0x00;
    TRISE = 0x00;
    LATE = 0x00;
    
    //INTURRUPT CONFIGURE
    RCONbits.IPEN = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    //TIMER1&CCP1 CONFIGURE
    T1CON = 0b00000010;
    CCP1CON = 0b00001000;
    CCPTMRS0 = 0x00;
    IPR1bits.TMR1IP = 0;
    IPR3bits.CCP1IP = 0;
    PIE3bits.CCP1IE = 1;
    PIE1bits.TMR1IE = 1;
    CCPR1X = (Servo1_on & 0xFF0000)/65536;
    
    //TIMER3&CCP4 CONFIGURE
    T3CON = 0b00000010;
    CCP4CON = 0b00001000;
    CCPTMRS1 = 0b00000001;
    IPR2bits.TMR3IP = 0;
    IPR4bits.CCP4IP = 0;
    PIE4bits.CCP4IE = 1;
    PIE2bits.TMR3IE = 1;
    CCPR4X = (Servo2_on & 0xFF0000)/65536;
    
    //TIMER0 CONFIGURE
//    T0CON = 0b01001000;
//    INTCON2bits.TMR0IP = 1;
//    INTCONbits.TMR0IE = 1;
    
    //I2C CONFIGURE
    SSP1CON1 = 0x28;
    SSP1CON2 = 0x00;
    SSP1STAT = 0x00;
    SSP1ADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
    TRISCbits.TRISC3 = 1; //SCL
    TRISCbits.TRISC4 = 1; //SDA
    
    //MPU6050 CONFIGURE
    __delay_ms(100);
    
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
    
    //USART CONFIGURE
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
    
    //START TIMERS
    T0CONbits.TMR0ON = 1;
    T1CONbits.TMR1ON = 1;
    T3CONbits.TMR3ON = 1;
    
}

void __interrupt() HiPriISR (void){
    while(1){
//            if(INTCONbits.TMR0IF){
//               TMR0handler();
//               continue;
//            }
    break;
    }
    return;
}
void __interrupt(low_priority) LoPriISR (void){
    Status_temp = STATUS;
    Wreg_temp = WREG;
    BSR_temp = BSR;
    while(1){
        if(PIR3bits.CCP1IF){
            CCP1handler();
            continue;
        }
        if(PIR4bits.CCP4IF){
            CCP4handler();
            continue;
        }
        if(PIR1bits.TMR1IF){
            TMR1handler();
            continue;
        }
        
        if(PIR2bits.TMR3IF){
            TMR3handler();
            continue;
        }

        break;
    }
    STATUS = Status_temp;
    WREG = Wreg_temp;
    BSR = BSR_temp;
}
void CCP1handler(void){
    unsigned long CCPRT;
    if(PIR1bits.TMR1IF){
        if(CCPR1H < 0b10000000){
            TMR1X++;
            time0counter++;
            PIR1bits.TMR1IF = 0;
        }
    }
    if(CCPR1X != TMR1X){
        PIR3bits.CCP1IF = 0;
        return;
    }
    LATDbits.LATD1 = !LATDbits.LATD1;
    if(LATDbits.LATD1 == 1){
        CCPRT = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        CCPRT += Servo1_on;
        CCPR1L = (CCPRT & 0x0000FF);
        CCPR1H = (CCPRT & 0x00FF00)/256;
        CCPR1X = (CCPRT & 0xFF0000)/65536;
    }
    if(LATDbits.LATD1 == 0){
      CCPRT = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        CCPRT += Servo1_off;
        CCPR1L = CCPRT & 0x0000FF;
        CCPR1H = (CCPRT & 0x00FF00)/256;
        CCPR1X = (CCPRT & 0xFF0000)/65536;
    }
    PIR3bits.CCP1IF = 0;
    return;
}
void CCP4handler(void){
    unsigned long CCPRT;
    if(PIR2bits.TMR3IF){
        if(CCPR4H < 0b10000000){
            TMR3X++;
            PIR2bits.TMR3IF = 0;
        }
    }
    if(CCPR4X != TMR3X){
        PIR4bits.CCP4IF = 0;
        return;
    }
    LATEbits.LATE1 = !LATEbits.LATE1;
    if(LATEbits.LATE1 == 1){
        CCPRT = ((unsigned long)CCPR4X*65536) + ((unsigned long)CCPR4H*256) + (unsigned long)CCPR4L;
        CCPRT += Servo2_on;
        CCPR4L = (CCPRT & 0x0000FF);
        CCPR4H = (CCPRT & 0x00FF00)/256;
        CCPR4X = (CCPRT & 0xFF0000)/65536;
    }
    if(LATEbits.LATE1 == 0){
        CCPRT = ((unsigned long)CCPR4X*65536) + ((unsigned long)CCPR4H*256) + (unsigned long)CCPR4L;
        CCPRT += Servo2_off;
        CCPR4L = (CCPRT & 0x0000FF);
        CCPR4H = (CCPRT & 0x00FF00)/256;
        CCPR4X = (CCPRT & 0xFF0000)/65536;
    }
    PIR4bits.CCP4IF = 0;
    return;
}
void TMR1handler(void){
    TMR1X++;
    time0counter++;
    PIR1bits.TMR1IF = 0;
    return;
}
void TMR3handler(void){
    TMR3X++;
    PIR2bits.TMR3IF = 0;
    return;
}
void TMR0handler(void){
    time0counter++;
    CurrentTime = time0counter*0.064;
    INTCONbits.TMR0IF = 0;
    return;
}

unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax){
    unsigned int first = (value - valuemin);
    unsigned long second = (unsigned long)first*((unsigned long)newmax - (unsigned long)newmin);
    unsigned int third = second/(valuemax - valuemin);
    return third + newmin;
   
}
void Servo1(unsigned int angle){
    unsigned int temp;
    if(angle < 0){
        angle = -1*angle;
    }
    if(angle > 110){
        angle = 110;
    }
    if(angle < 70){
        angle = 70;
    }
    temp = Servo1_on;
    Servo1_on = map(angle,0,180,2000,9600);//10000 seems to be max 2000 seems to be the min
    Servo1_off -= Servo1_on - temp;
}
void Servo2(unsigned int angle){
    unsigned int temp;
    if(angle < 0){
        angle = -1*angle;
    }
    if(angle > (50+20)){
        angle = 110;
    }
    if(angle < (50-20)){
        angle = 70;
    }
    temp = Servo2_on;
    Servo2_on = map(angle,0,180,2000,9600);//10000 seems to be max 2000 seems to be the min
    Servo2_off -= Servo2_on - temp;
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

void I2C_Master_Stop(void){
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

void MPU6050_Read(void){
    char buffer[40];
    int Ax,Ay,Az,Gx,Gy,Gz,T;
    float accAngleX, accAngleY;
    float AX,AY,AZ,GX,GY,GZ,t, roll, pitch;
    CurrentTime = ((time0counter*0.016384)+(((double)TMR1H*256) + (double)TMR1L)*0.000000250)*1000;
    float elapsedTime = (CurrentTime - previousTime) / 1000;
    previousTime = CurrentTime;
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
//    sprintf(buffer,"Ax = %.2f \t",AX);	
//    UART_Write_String(buffer);
//   
//    sprintf(buffer," Ay = %.2f \t",AY);
//    UART_Write_String(buffer);
//		
//    sprintf(buffer," Az = %.2f \t",AZ);
//    UART_Write_String(buffer);
//
//    sprintf(buffer," T = %.2f  ",t);
//    UART_Write_String(buffer);
//
//    sprintf(buffer," Gx = %.2f \t",GX);
//    UART_Write_String(buffer);
//
//    sprintf(buffer," Gy = %.2f \t",GY);
//    UART_Write_String(buffer);
//   
//    sprintf(buffer," Gz = %.2f\r\n",GZ);
//    UART_Write_String(buffer);
    
      //accAngleX = (atan(AX / sqrt(pow(AX, 2) + pow(AZ, 2))) * 180 / 3.14159265359);
      accAngleX=(-atan2(AX/9.8,-AY/9.8)/2/3.141592654*360)+1.9;
//    //accAngleY = (atan(-1 * AX / sqrt(pow(AY, 2) + pow(AZ, 2))) * 180 / 3.14159265359);
     accAngleY = (-atan2(-AZ/9.8,-AY/9.8)/2/3.141592654*360)-1.9;
    GyroAngleX = GX*elapsedTime;
    GyroAngleY = -GZ*elapsedTime;
//    yaw += GZ*elapsedTime;
    roll = 0.96*(GyroAngleX + accAngleX) + 0.04*accAngleX;
    pitch = 0.96*(GyroAngleY+ accAngleY) + 0.04*accAngleY;
//    
    sprintf(buffer," roll = %.2f \t",roll);
    UART_Write_String(buffer);
    sprintf(buffer," pitch = %.2f \t",pitch);
    UART_Write_String(buffer);
    sprintf(buffer," Servo1 Command = %.2f \t",90.0-roll);
    UART_Write_String(buffer);
    sprintf(buffer," Servo2 Command = %.2f \r\n",50.0-pitch);
    UART_Write_String(buffer);
    
    Servo1(90.0-roll);
    Servo2(50.0-pitch);
    
    return;
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

