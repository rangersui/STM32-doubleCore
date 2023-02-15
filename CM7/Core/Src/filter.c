/**

    @file filter.c
    @brief This file provides a simple Butterworth low-pass filter function for rocessing ADC data.
    @attention
    This software is licensed under terms that can be found in the LICENSE file
    in the root directory of this software component.
    If no LICENSE file comes with this software, it is provided AS-IS.
*/
#include "filter.h"
/**
 * @brief linear interpolation function
 * 
 * @param array 
 * @param array_len 
 * @param converted_array 
 * @param converted_array_len 
 */
void Linear_Interpolation(uint16_t *array, size_t array_len, uint32_t *converted_array, size_t converted_array_len)
{
  // calculate step size
  double step = (double)(array_len - 1) / (double)(converted_array_len - 1);

  // perform linear interpolation
  for (size_t i = 0; i < converted_array_len; i++)
  {
    double index = i * step;
    size_t idx1 = (size_t)index;
    size_t idx2 = idx1 + 1;
    double frac = index - idx1;
    converted_array[i] = (uint16_t)((1.0 - frac) * array[idx1] + frac * array[idx2]);
  }
}
/**
 * @brief Butterworth low-pass filter coefficients for 5000Hz cutoff frequency
 *
 * @param src_arr
 * @param dest_array
 */
// void ButterLPF_5000Hz(float *src_arr, float *dest_array)
// {
//     for (int n = 0; n < ADC_SIZE; n++)
//     {
//         if (n == 0)
//         {
//             dest_array[n] = b[0] * src_arr[n];
//         }
//         else
//         {
//             dest_array[n] = b[0] * src_arr[n] + b[1] * src_arr[n - 1] - a[1] * dest_array[n - 1];
//         }
//     }
// }