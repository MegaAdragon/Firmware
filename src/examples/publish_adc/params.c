#include <systemlib/param/param.h>

// 16 is max name length

/**
 * Sonar standard deviation.
 *
 * @group SONAR
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SNR_DEV, 0.05f);

/**
 * Length of moving average filter
 *
 * @group SONAR
 * @min 0
 * @max 20
 */
PARAM_DEFINE_INT32(SNR_FIL_LEN, 10);

/**
 * ADC Channel
 *
 * @group SONAR
 * @min 1
 * @max 20
 */
PARAM_DEFINE_INT32(SNR_ADC_CH, 14);
