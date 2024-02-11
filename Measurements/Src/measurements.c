#include "stdint.h"
#include "math.h"

#include "measurements.h"

#define ADC_RES_Current 4096U
#define ADC_Ref_Current 3.3f

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
  * @brief  RMS value calculation
  * 
  * @param y Pointer to samples buffer
  * @param size Number of samples to calculate
  * @retval Signal RMS value
  */
float RMS_Calculate(const float *y, uint16_t size);

/**
  * @brief  Get signal samples for one channel from n channels samples
  * 
  * @param Chn Channels amount
  * @param Chx Channel x number
  * @param Samples Pointer to all channels samples
  * @param x Pointer to channel samples
  * @param n Amount of samples for one channel
  */
void Samples_getSample(const uint8_t Chn, const uint8_t Chx, const Sample_t *Samples, Sample_t *x, const uint16_t n);












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

/**
  * @brief  RMS value calculation
  * 
  * @param y Pointer to samples buffer
  * @param size Number of samples to calculate
  * @retval Signal RMS value
  */
float RMS_Calculate(const float *y, uint16_t size)
{
  uint16_t i;
  float RMS = 0.0f;
  float Sum = 0.0f;

  for(i=0; i<size; i++)
  {
    Sum = Sum + y[i] * y[i];
  }

  if(size > 0)
  {
    RMS = sqrtf((Sum/(float)size));
  }
  else
  {
    ;
  }

  return RMS;

}

/**
  * @brief  Get signal samples for one channel from n channels samples
  * 
  * @param Chn Channels amount
  * @param Chx Channel x number
  * @param Samples Pointer to all channels samples
  * @param x Pointer to channel samples
  * @param n Amount of samples for one channel
  */
void Samples_getSample(const uint8_t Chn, const uint8_t Chx, const Sample_t *Samples, Sample_t *x, const uint16_t n)
{
  uint16_t i;

  for(i=0; i<n; i++)
  {
    *(x + i) = *(Samples + i*Chn + Chx);
  }
}

/**
  * @brief  Get current signal RMS value
  * @retval Signal RMS value
  */
float Measure_getCurrentRMS(void)
{
  uint16_t n = SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS_RMS;
  Sample_t x[n];
  float y[n];
  float RMS;
  uint16_t i;

  // Get current signal samples from Samples buffer
  // for(i=0; i<n; i++)
  // {
  //   x[i] = Samples[i][Current];
  // }
  Samples_getSample(AMOUNT_OF_MEASUREMENT_CHANNELS, Current, Samples[0], x, n);

  // Get real signal value samples
  for(i=0; i<n; i++)
  {
    y[i] = SigRealVal(
      ADC_RawToReal(x[i], ADC_RES_Current, ADC_Ref_Current),
      1U
    );
  }

  // Calculate RMS
  RMS = RMS_Calculate(y, n);

  return RMS;
}