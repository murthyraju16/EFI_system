/******************************************************************************
*                      Includes
******************************************************************************/

#include<avr/io.h>
#include"EFI_lcd.h"
#include<util/delay.h>
#include<math.h>

/******************************************************************************
*                      Defines and data types
******************************************************************************/

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

void enable_lcd_command()
{
    CLR_BIT(PORTD,PD1);
    SET_BIT(PORTD,PD0); /*To enable the lcd panel */
    //_delay_ms(10);
    CLR_BIT(PORTD,PD0);
}

void enable_lcd_data()
  {
    SET_BIT(PORTD,PD1);
    SET_BIT(PORTD,PD0);
    //_delay_ms(10);
    CLR_BIT(PORTD,PD0);
  }

void floatTointstrg(float AFR_val, char *res, int afterpoint)
  {
      /* Extract integer part*/
      int intpart = (int)AFR_val;

      /* Extract floating part */
      float fltpart = AFR_val - (float)intpart;

      /* convert integer part to string*/
      int i = intToStr(intpart, res, 0);

      /* check for display option after point*/
      if (afterpoint != 0)
      {
          res[i] = '.';  /* add dot*/

          /* Get the value of fraction part upto given no of points after dot*/
          fltpart = fltpart * pow(10, afterpoint);

          intToStr((int)fltpart, res + i + 1, afterpoint);
      }
  }

int intToStr(int val, char str[], int d)
  {
      int i = 0;
      while (val)
      {
          str[i++] = (val%10) + '0';
          val = val/10;
      }

      // If number of digits required is more, then
      // add 0s at the beginning
      while (i < d)
          str[i++] = '0';

      str_reverse(str, i);
      str[i] = '\0';
      return i;
}
void str_reverse(char *str, int len)
 {
      int i=0, j=len-1, temp;
      while (i<j)
      {
          temp = str[i];
          str[i] = str[j];
          str[j] = temp;
          i++;
          j--;
      }
 }

/******************************************************************************
*                      Internal functions
******************************************************************************/

/******************************************************************************
*                      End of File
******************************************************************************/




