/**
    @file filter.h
    @brief Header file for the filter.c file
    @copyright (c) [Copyright]
    Licensed under the GPL license.
    See LICENSE file in the project root for full license information.
*/

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------/
/ USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------/
/ USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------/
/ USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/**

    @brief Butterworth low-pass filter for 5000Hz.
    @param src_arr: pointer to the source array
    @param dest_arr: pointer to the destination array
    @retval None
*/
void ButterLPF_5000Hz(float *src_arr, float *dest_arr);
void Linear_Interpolation(uint16_t *array, size_t array_len, uint32_t *converted_array, size_t converted_array_len);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */