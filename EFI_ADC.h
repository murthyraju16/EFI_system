#ifndef EFI_ADC_H
#define EFI_ADC_H

/******************************************************************************
*                      Includes
******************************************************************************/
#include<avr/interrupt.h>
#include<stdint.h>
#include<avr/io.h>
/******************************************************************************
*                      Defines and data types
******************************************************************************/
#define ENABLE_ADC() ADCSRA|=(1<<ADEN); /* enabling the ADC */
#define DISABLE_ADC() ADCSRA&=~(1<<ADEN); /* disable the ADC */
/******************************************************************************
*                      Global variables
******************************************************************************/
/******************************************************************************
*                      Static variables
******************************************************************************/
/******************************************************************************
*                      Internal function prototypes
******************************************************************************/


/******************************************************************************
*                      Public functions
******************************************************************************/
extern void ADC_setup();
extern uint16_t Read_ADC(uint8_t ADCchannel);
#endif /* EFI_ADC_H*/
/******************************************************************************
*                      End of File
******************************************************************************/