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



/**
  * @brief  Conversion from Raw ADC value to real value
  * 
  * @param RawVal Raw value from ADC
  * @param Res ADC resolution 2^n, ex. 4096
  * @param Ref ADC reference voltage, ex. 3.3 [V]
  * @retval Real voltage value on ADC pin, ex. 2.5 [V]
  */
float ADC_RawToReal(const Sample_t RawVal, const int16_t Res, const float Ref);


/**
  * @brief  Conversion from Real ADC value to real signal value
  * 
  * @param ADC_RealVal Real voltage on ADC input, ex. 2.5 [V]
  * @param SigDiv Signal divider, ex. 12.0f
  * @retval Real signal value, ex. 110 [V]
  */
float SigRealVal(float ADC_RealVal, float SigDiv);















/**
  * @brief  Conversion from Raw ADC value to real value
  * 
  * @param RawVal Raw value from ADC
  * @param Res ADC resolution 2^n, ex. 4096
  * @param Ref ADC reference voltage, ex. 3.3 [V]
  * @retval Real voltage value on ADC pin, ex. 2.5 [V]
  */
float ADC_RawToReal(const Sample_t RawVal, const int16_t Res, const float Ref)
{
    if (Res > 0)
    {
        return ((float)RawVal * Ref) / Res;
    }
    else
    {

    }
}

/**
  * @brief  Conversion from Real ADC value to real signal value
  * 
  * @param ADC_RealVal Real voltage on ADC input, ex. 2.5 [V]
  * @param SigDiv Signal divider, ex. 12.0f
  * @retval Real signal value, ex. 110 [V]
  */
float SigRealVal(float ADC_RealVal, float SigDiv)
{
  return ADC_RealVal * SigDiv;
}