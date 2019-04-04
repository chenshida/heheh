
#ifndef TEMPERATUREMONITOR_HELPER_H
#define TEMPERATUREMONITOR_HELPER_H



float NTCConvert(uint16_t ADCValue);
float getVoltage(uint16_t ADCValue);
int tolower(int c);
int htoi(char s[]);

#endif //TEMPERATUREMONITOR_HELPER_H
