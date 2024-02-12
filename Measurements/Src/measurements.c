#include "stdint.h"
#include "math.h"

#include "measurements.h"


typedef enum
{
    Current = 0U,
    Voltage_Negative = 1U,
    Voltage_Positive = 2U

}ADC_Channels_Mapping_t;

typedef const struct
{
  uint8_t Chx;  /* Channel number */
  uint16_t Res; /* ADC resolution */
  float Ref;    /* ADC voltage reference */
  float Divider;/* Voltage divider */
}ADC_Ch_Param_t;


/**
  * @brief  Measurements ADC Samples array
  */
Sample_t Samples[SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS][AMOUNT_OF_MEASUREMENT_CHANNELS] = {0};

/**
  * @brief  ADC channels parameter array
  */
ADC_Ch_Param_t ADC_Chx_Param[AMOUNT_OF_MEASUREMENT_CHANNELS] = 
{
  {Current, 4096U, 3.3f, 100U},
  {Voltage_Negative, 4096U, 3.3f, 100U},
  {Voltage_Positive, 4096U, 3.3f, 100U}
};

/**
  * @brief  Measurement watch list
  */
Measure_Watch_List_t Measure_Watch_list = 
{
  .Vrms = 0.0f,
  .Irms = 0.0f,
  .FRQ = 0.0f,
  .Vpp = 0.0f,
  .Ipp = 0.0f
};

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
void Samples_getChxRawSamples(const uint8_t Chn, const uint8_t Chx, const Sample_t *Samples, Sample_t *x, const uint16_t n);

/**
  * @brief  Calculate signal real value
  * 
  * @param Chx Channel x number
  * @param x Pointer to channel raw samples
  * @param x Pointer to channel real samples
  * @param n Amount of samples
  */
void Samples_CalcRealValue(const uint8_t Chx, const Sample_t *x, float *y, const uint16_t n);

/**
  * @brief  Calculate two real value difference
  * 
  * @param y1 Pointer to channel y1 real samples
  * @param y2 Pointer to channel y2 real samples
  * @param y Pointer to difference real samples
  * @param n Amount of samples
  */
void Samples_CalcDiff(const float *y1, const float *y2, float *y, const uint16_t n);









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
void Samples_getChxRawSamples(const uint8_t Chn, const uint8_t Chx, const Sample_t *Samples, Sample_t *x, const uint16_t n)
{
  uint16_t i;

  for(i=0; i<n; i++)
  {
    *(x + i) = *(Samples + i*Chn + Chx);
  }
}

/**
  * @brief  Calculate signal real value
  * 
  * @param Chx Channel x number
  * @param x Pointer to channel raw samples
  * @param x Pointer to channel real samples
  * @param n Amount of samples
  */
void Samples_CalcRealValue(const uint8_t Chx, const Sample_t *x, float *y, const uint16_t n)
{
  uint16_t i;

  for(i=0; i<n; i++)
  {
    y[i] = SigRealVal(
      ADC_RawToReal(x[i], ADC_Chx_Param[Chx].Res, ADC_Chx_Param[Chx].Ref),
      ADC_Chx_Param[Chx].Divider
    );
  }
}

/**
  * @brief  Calculate two real value difference
  * 
  * @param y1 Pointer to channel y1 real samples
  * @param y2 Pointer to channel y2 real samples
  * @param y Pointer to difference real samples
  * @param n Amount of samples
  */
void Samples_CalcDiff(const float *y1, const float *y2, float *y, const uint16_t n)
{
  uint16_t i;

  for(i=0; i<n; i++)
  {
    y[i] = y1[i] - y2[i];
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

  Samples_getChxRawSamples(AMOUNT_OF_MEASUREMENT_CHANNELS, ADC_Chx_Param[Current].Chx, Samples[0], x, n);
  Samples_CalcRealValue(ADC_Chx_Param[Current].Chx, x, y, n);

  return RMS_Calculate(y, n);
}

/**
  * @brief  Get voltage signal RMS value. Two ADC channels are used for measuremt positive and negative sine value separetly.
  * @retval Signal RMS value
  */
float Measure_getVoltageRMS(void)
{
  uint16_t n = SAMPLES_AMOUNT_PER_ONE_PERIOD * AMOUNT_OF_PERIODS_RMS;
  Sample_t xp[n], xn[n];
  float yp[n], yn[n], ydiff[n];

  Samples_getChxRawSamples(AMOUNT_OF_MEASUREMENT_CHANNELS, ADC_Chx_Param[Voltage_Positive].Chx, Samples[0], xp, n);
  Samples_getChxRawSamples(AMOUNT_OF_MEASUREMENT_CHANNELS, ADC_Chx_Param[Voltage_Negative].Chx, Samples[0], xn, n);

  Samples_CalcRealValue(ADC_Chx_Param[Voltage_Positive].Chx, xp, yp, n);
  Samples_CalcRealValue(ADC_Chx_Param[Voltage_Positive].Chx, xn, yn, n);

  Samples_CalcDiff(yp, yn, ydiff, n);

  return RMS_Calculate(ydiff, n);
}