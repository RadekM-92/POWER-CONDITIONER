#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

#define SAMPLES_AMOUNT_PER_ONE_PERIOD 128U
#define AMOUNT_OF_PERIODS 3U
#define AMOUNT_OF_MEASUREMENT_CHANNELS 3U
#define AMOUNT_OF_ALL_SAMPLES SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS * AMOUNT_OF_MEASUREMENT_CHANNELS
#define AMOUNT_OF_PERIODS_RMS 2U

typedef uint32_t Sample_t;
typedef struct{
    float Vrms;
    float Irms;
    float FRQ;
    float Vpp;
    float Ipp;
}Measure_Watch_List_t;

extern Sample_t Samples[SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS][AMOUNT_OF_MEASUREMENT_CHANNELS];
extern Measure_Watch_List_t Measure_Watch_list;

extern float Measure_getCurrentRMS(void);
extern float Measure_getVoltageRMS(void);

#endif