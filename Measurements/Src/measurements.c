#include "stdint.h"

#include "measurements.h"

typedef enum
{
    Current = 0U,
    Voltage_Negative = 1U,
    Voltage_Positive = 2U

}ADC_Channels_Mapping_t;

/**
  * @brief  Measurements ADC Samples array
  */
Sample_t Samples[SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS][AMOUNT_OF_MEASUREMENT_CHANNELS] = {0};