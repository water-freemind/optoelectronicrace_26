#include "ADC.h"
#include "string.h"
uint16_t ADC_VALUE[40];

unsigned int adc_getValue(unsigned int number)
{
    unsigned int sum = 0;
    unsigned char i;

    for (i = 0; i < number; i++)
    {
        ADC_VALUE[0] = 0;
        __WFI();
        while (ADC_VALUE[0] == 0) {
            __WFI();
        }
        sum += ADC_VALUE[0];
    }

    return sum / number;
}
