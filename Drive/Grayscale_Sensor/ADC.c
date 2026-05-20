#include "ADC.h"
#include "string.h"
uint16_t ADC_VALUE[40];

unsigned int adc_getValue(unsigned int number)
{
    volatile uint16_t *p = (volatile uint16_t *)&ADC_VALUE[0];
    unsigned int sum = 0;
    unsigned char i;

    for (i = 0; i < number; i++)
    {
        *p = 0;
        while (*p == 0);
        sum += *p;
    }

    return sum / number;
}
