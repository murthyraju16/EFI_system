  /******************************************************************************
* File Name: main.c
* Description:  This file contains functions for Electronic fuel injection system
* Tool-Chain: AVR
*
*  Modification History:
*  Created by:      Yenugudhati Murthy Raju(141705)

*  Description:     Pins that I used  for Electronic fuel injection system
PD3 --> Engine ON
PC0 --> Manifold Pressure Sensor(Engine load)
PC1 --> Engine Speed Sensor(RPM)
PC2 --> Coolant Temperature sensor(degree celsius)
PC3 --> Exhaust gas sensor
PD6 --> Fuel injection duration as PWM pulse width     OCOA=PD6


Description of Workflow :
1)start engine.
2)Read the ADC value through PC1 from Speed Sensor.
3)Read the ADC value through PC0 from MAP sensor for engine load.
4)Read the ADC value through PC2 from Coolant temperature sensor.
5)Read the ADC value through PC3 from exhaust gas oxygen sensor.
6)Timing of fuel injector is shown in the lcd and those are in milli-seconds.
7)Along with that, timing is shown as PWM with open timing as the duty cycle. Here I consider the time period for each pulse is 80 milliseconds.

/******************************************************************************
*                      Includes
******************************************************************************/
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<stdint.h>
#include"EFI_lcd.h"
#include"EFI_ADC.h"
#include"EFI_PWM.h"

/******************************************************************************
*                      Defines and data types
******************************************************************************/
#define SET_BIT(PORT,BIT) PORT|=(1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&=~(1<<BIT)
#define READ_BIT(PORT,BIT) PORT & (1<<BIT)


/*Defining input and output ports*/
#define ENGINE PD3

typedef struct
{
    volatile unsigned int BIT_FLAG:1;

}FLAG;

FLAG  ENGINE_flag;

static uint8_t rpm_index(float);
static uint8_t load_index(float);
static uint8_t temp_index(float);
static uint8_t oxy_index(float);

/******************************************************************************
*                    Main Function
******************************************************************************/

int main()
{
    CLR_BIT(DDRD,ENGINE);
    SET_BIT(PORTD,ENGINE);

    CLR_BIT(DDRC,PC0);
    CLR_BIT(DDRC,PC1);
    CLR_BIT(DDRC,PC2);
    CLR_BIT(DDRC,PC3);

    SET_BIT(DDRD,PD6);

    // In Look up table of Fuel timing map the row is for RPM and column id for Load
    int Fuel_Map[16][16]= {{15.1, 14.9, 14.8, 14.6, 14.4, 13.9, 13.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5,12.5, 12.5, 12.5},
                           {15.1, 14.9, 14.7, 14.3, 14.3, 13.7, 13.5, 12.7, 12.7, 12.7,12.7, 12.7, 12.7, 12.7, 12.7, 12.7},
                           {15.1, 14.9, 14.7, 14.0, 13.9, 13.6, 13.5, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9},
                           {15.1, 14.7, 14.3, 13.9, 13.8, 13.6, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.4, 13.4, 13.4},
                           {15.1, 14.7, 13.9, 13.9, 13.8, 13.5, 13.4, 13.4, 13.4, 13.4, 13.2, 13.2, 13.2, 13.2, 13.2, 13.2},
                           {15.1, 14.7, 13.9, 13.8, 13.8, 13.5, 13.3, 12.7, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5},
                           {14.1, 14.1, 13.9, 13.8, 13.7, 13.4, 13.3, 12.7, 12.5, 12.5, 12.4, 12.3, 12.3, 12.2, 12.0, 12.0},
                           {13.9, 13.9, 13.7, 13.7, 13.5, 13.3, 13.2, 12.6, 12.2, 11.3, 11.2, 11.2, 11.2, 11.1, 11.1, 11.0},
                           {13.8, 13.6, 13.5, 13.5, 13.4, 13.2, 13.2, 12.6, 12.1, 11.3, 11.2, 11.2, 11.2, 11.1, 11.0, 10.9},
                           {13.5, 13.5, 13.3, 13.2, 13.2, 13.1, 13.1, 12.6, 12.0, 11.2, 11.1, 11.1, 11.1, 11.0, 10.9, 10.6},
                           {13.5, 13.4, 13.2, 13.1, 13.1, 13.0, 12.9, 12.5, 12.0, 11.2, 11.1, 11.1, 11.0, 10.9, 10.6, 10.5},
                           {13.3, 13.2, 13.2, 13.1, 13.0, 12.9, 12.8, 12.5, 11.9, 11.1, 11.0, 11.0, 11.0, 10.8, 10.4, 10.4},
                           {13.2, 13.1, 13.2, 13.1, 13.0, 12.8, 12.8, 12.2, 11.8, 11.1, 11.0, 10.9, 10.9, 10.7, 10.3, 10.2},
                           {13.1, 13.1, 13.0, 13.0, 12.9, 12.6, 12.3, 11.9, 11.7, 11.0, 10.9, 10.8, 10.7, 10.6, 10.2, 10.2},
                           {13.1, 13.1, 12.9, 12.9, 12.8, 12.4, 12.3, 11.9, 11.7, 10.9, 10.8, 10.7, 10.6, 10.5, 10.1, 10.0},
                           {12.9, 12.9, 12.9, 12.8, 12.6, 12.3, 12.0, 11.9, 11.7, 10.8, 10.7, 10.6, 10.5, 10.4, 10.0, 10.0}};
    /* Look up table for Engine temperature sensor */
    float temp_factor[5]={1.2,1.15,1.0,0.99,0.98};
    /* Look up table for EGO sensor */
    float oxy_factor[7] = { 1.0, 1.0, 0.97, 0.94, 0.9, 0.83, 0.75 };
    float Engine_load,Engine_speed,coolant_temp,EGO_level;
    int load_ind,rpm_ind,temp_ind,oxy_ind;
    float base_PW,pulse_width,AFR_value;
    char a[200];
    char b[11]="ENGINE_OFF";
    int i;

    ADC_setup();
    ENABLE_ADC();
    PWM_setup();
    EICRA = ~(1<<ISC11)|~(1<<ISC10); /* enable the interrupt and it is a Falling edge trigger */
    EIMSK = (1<<INT1);

    sei();

    PORTB=0x38;
    enable_lcd_command();
    PORTB=0x0E;
    enable_lcd_command();

    ENGINE_flag.BIT_FLAG=0;

    while(1)
    {
        if(!(READ_BIT(PIND,ENGINE)))
        {
            ENGINE_flag.BIT_FLAG=0;
            PWM_enable();
            Engine_load=(Read_ADC(0)*(230.0/1023.0))+20.0;                /* map = 20 to 250 kPa */
            load_ind=load_index(Engine_load);

            Engine_speed=Read_ADC(1)*(8000.0/1023.0);                   /* rpm = 0-8000 */
            rpm_ind=rpm_index(Engine_speed);

            coolant_temp=(Read_ADC(2)*(110.0/1023.0))+273.0;             /* temp = 273k to 383k*/ /*--considering the india temperature--*/
            temp_ind=temp_index(coolant_temp);

            EGO_level=(Read_ADC(3)*(0.6/1023.0))+0.2;                    /* ego  = 0.2 to 0.8*/
            oxy_ind=oxy_index(EGO_level);


            PORTB=0x01;
            enable_lcd_command();
            PORTB=0x80;
            enable_lcd_command();

            AFR_value = Fuel_Map[rpm_ind][load_ind];                    /* depending on the speed and load on engine , the base pulse width is identified from look-up table of fuel-timing map */
            base_PW =  2.0 / (4.0 * AFR_value);
            pulse_width = base_PW*temp_factor[temp_ind]*oxy_factor[oxy_ind]; /* effect on injection timing by engine temperature and o2 level in exhaust gas. */

            PWM_EFI(pulse_width); /* generating pwm signal depends on the engine speed,temperature,o2 level and engine load*/

            floatTointstrg(pulse_width*1000,a,2);
        }
        else
        {
            PWM_disable();
            PORTB=0x01;
            enable_lcd_command();
            PORTB=0x80;
            enable_lcd_command();
            for(i=0;b[i]!='\0';i++)
            {
                a[i]=b[i];
            }

        }
        for(i=0;a[i]!='\0';i++)
        {
            PORTB=a[i];
            enable_lcd_data();
        }
        _delay_ms(100);
  }

}

/******************************************************************************
*                     Interrupt Service Routines
******************************************************************************/

/*ISR(INT1_vect)
{
    cli();
    ENGINE_flag.BIT_FLAG==1;
    sei();

}*/

/*********************************************************************************************************************
*           Index's for look up tables of fuel - timing map,Engine temperature sensor,EGO sensor                     *
*********************************************************************************************************************/

static uint8_t rpm_index(float value)
{
    uint8_t index;
    if ((value>=0) && (value<350))
    {
        index=0;
    }
    else if ((value>=350) && (value < 750))
    {
        index=1;

    }
    else if ((value>= 750) &&(value < 1000))
    {
        index=2;

    }
    else if ((value>= 1000) && (value < 1500))
    {
        index=3;
    }
    else if ((value >= 1500) && (value < 2000))
    {
        index=4;
    }
    else if ((value >= 2000) && (value < 2500))
    {
        index=5;
    }
    else if ((value >= 2500) && (value < 3000))
    {
        index=6;
    }
    else if ((value >= 3000) && (value < 3500))
    {
        index=7;
    }
    else if ((value >= 3500) && (value < 4000))
    {
        index=8;
    }
    else if ((value >= 4000) && (value < 4500))
    {
        index=9;
    }
    else if ((value >= 4500) && (value < 5000))
    {
        index=10;
    }
    else if ((value >= 5000) && (value < 5500))
    {
        index=11;
    }
    else if ((value >= 5500) && (value < 6000))
    {
        index=12;
    }
    else if ((value >= 6000) && (value < 6750))
    {
        index=13;
    }
    else if ((value >= 6750) && (value < 7500))
    {
        index=14;
    }
    else if ((value >= 7500) && (value <= 8000))
    {
        index=15;
    }
    return index;
}

static uint8_t load_index(float value)
{
    uint8_t index;
    if ((value>=20) && (value<35))
    {
        index=0;
    }
    else if ((value>=35) && (value < 50))
    {
        index=1;

    }
    else if ((value>= 50) &&(value < 65))
    {
        index=2;

    }
    else if ((value>= 65) && (value < 70))
    {
        index=3;
    }
    else if ((value >= 70) && (value < 85))
    {
        index=4;
    }
    else if ((value >= 85) && (value < 100))
    {
        index=5;
    }
    else if ((value >= 115) && (value < 130))
    {
        index=6;
    }
    else if ((value >= 130) && (value < 145))
    {
        index=7;
    }
    else if ((value >= 145) && (value < 160))
    {
        index=8;
    }
    else if ((value >= 160) && (value < 175))
    {
        index=9;
    }
    else if ((value >= 175) && (value < 190))
    {
        index=10;
    }
    else if ((value >= 190) && (value < 205))
    {
        index=11;
    }
    else if ((value >= 205) && (value < 220))
    {
        index=12;
    }
    else if ((value >= 220) && (value < 230))
    {
        index=13;
    }
    else if ((value >= 230) && (value < 240))
    {
        index=14;
    }
    else if ((value >= 240) && (value <= 250))
    {
        index=15;
    }
    return index;
}

static uint8_t temp_index(float value)
{
    uint8_t index;
    if ((value>=273.0) && (value<295.0))
    {
        index=0;
    }
    else if ((value>=295.0) && (value < 317.0))
    {
        index=1;

    }
    else if ((value>= 317.0) &&(value < 339.0))
    {
        index=2;

    }
    else if ((value>= 339.0) && (value < 361.0))
    {
        index=3;
    }
    else if ((value >= 361.0) && (value <= 383.0))
    {
        index=4;
    }
    return index;
}

static uint8_t oxy_index(float value)
{
    uint8_t index;
    if ((value>=0.2) && (value<0.35))
    {
        index=0;
    }
    else if ((value>=0.35) && (value <= 0.45))
    {
        index=1;

    }
    else if ((value> 0.45) &&(value < 0.55))
    {
        index=2;

    }
    else if ((value>= 0.55) && (value < 0.65))
    {
        index=3;
    }
    else if ((value >= 0.65) && (value < 0.7))
    {
        index=4;
    }
    else if ((value >= 0.7) && (value < 0.75))
    {
        index=5;
    }
    else if ((value >= 0.75) && (value <= 0.8))
    {
        index=6;
    }
    return index;
}

/******************************************************************************
*                      End of File
******************************************************************************/






