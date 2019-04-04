#include "stdint.h"
#include "helper.h"
#include "math.h"

static float Bx = 3950.0;
static float T2 =(273.15+25.0);
static float Rp=10000.0;
static float Ka = 273.15;

float NTCConvert(uint16_t ADCValue)
{
    float voltage_ri;
    float Ri;
    float temp_deg;
    float ADCVREF=3.3;
    voltage_ri = (float)((ADCValue * ADCVREF) / 4095);
    Ri = (float) ((voltage_ri * Rp) / (ADCVREF - voltage_ri));
    temp_deg = (float) ((1 / (log(Ri / Rp) / Bx + (1 / T2))) - Ka + 0.5);
    return temp_deg;
}

float getVoltage(uint16_t ADCValue)
{
    float voltage_tmp=0;
    float ADCVREF=3.3;
    voltage_tmp = (float)(ADCValue*ADCVREF/4096);
    return voltage_tmp;
}

int tolower(int c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return c + 'a' - 'A';
    }
    else
    {
        return c;
    }
}

int htoi(char s[])
{
    int i;
    int n = 0;
    if (s[0] == '0' && (s[1]=='x' || s[1]=='X'))
    {
        i = 2;
    }
    else
    {
        i = 0;
    }
    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)
    {
        if (tolower(s[i]) > '9')
        {
            n = 16 * n + (10 + tolower(s[i]) - 'a');
        }
        else
        {
            n = 16 * n + (tolower(s[i]) - '0');
        }
    }
    return n;
}