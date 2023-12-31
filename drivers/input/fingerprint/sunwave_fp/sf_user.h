#ifndef __SF_USER_H__
#define __SF_USER_H__

#include "sf_def.h"

//-----------------------------------------------------------------------------

// platform select
#define SF_PLATFORM_SEL             SF_TEE_QSEE
// compatible select
#define SF_COMPATIBLE_SEL           SF_COMPATIBLE_NOF
// power mode select
#define SF_POWER_MODE_SEL           PWR_MODE_GPIO
// debug log select
#define SF_LOG_ENABLE               1
//-----------------------------------------------------------------------------

/* Dts node. */
#if 0
#define COMPATIBLE_SW_FP            "fp,fingerprint"

// for not mtk
#define COMPATIBLE_RESET_GPIO       "fp,gpio_reset"
#define COMPATIBLE_IRQ_GPIO         "fp,gpio_irq"
#define COMPATIBLE_PWR_GPIO         "fp,gpio_pwr"

//for mtk pinctl system
#define FINGER_POWER_ON             "finger_power_high"
#define FINGER_POWER_OFF            "finger_power_low"
#define FINGER_RESET_LOW            "fp_reset_reset"
#define FINGER_RESET_HIGH           "fp_reset_active"
#define FINGER_INT_SET              "fp_irq_active"
#else
#define COMPATIBLE_SW_FP            "sunwave,fingerprint"

// for not mtk
#define COMPATIBLE_RESET_GPIO       "sunwave,gpio_reset"
#define COMPATIBLE_IRQ_GPIO         "sunwave,gpio_irq"
#define COMPATIBLE_PWR_GPIO         "sunwave,gpio_pwr"

//for mtk pinctl system
#define FINGER_POWER_ON             "sunwave_pwr_high"
#define FINGER_POWER_OFF            "sunwave_pwr_low"
#define FINGER_RESET_LOW            "sunwave_reset_reset"
#define FINGER_RESET_HIGH           "sunwave_reset_active"
#define FINGER_INT_SET              "sunwave_irq_active"
#endif /*CONFIG_FINGERPRINT_VENDOR_CHECK*/

/* regulator VDD select */
#define SF_VDD_NAME                 "vdd"
#define SF_VDD_MIN_UV               2800000
#define SF_VDD_MAX_UV               2800000
//-----------------------------------------------------------------------------

#define ANDROID_WAKELOCK            1
#define SF_INT_TRIG_HIGH            0
//-----------------------------------------------------------------------------

//for mtk6739 kernel4.4 spi speed mode
#define MTK_6739_SPEED_MODE         0
//-----------------------------------------------------------------------------
#if (SF_PLATFORM_SEL == SF_REE_MTK_L5_X)
//android mtk androidL 5.x none dts config file
#define MTK_L5_X_POWER_ON           1
#define MTK_L5_X_IRQ_SET            0

//power GPIO
#if MTK_L5_X_POWER_ON
#define  GPIO_SW_PWR_PIN            GPIO_FIGERPRINT_PWR_EN_PIN
#define  GPIO_SW_PWR_M_GPIO         GPIO_MODE_00
#endif

//interrupt pin
#define  GPIO_SW_IRQ_NUM            CUST_EINT_FIGERPRINT_INT_NUM
#define  GPIO_SW_INT_PIN            GPIO_FIGERPRINT_INT

//reset pin
#define  GPIO_SW_RST_PIN            GPIO_FIGERPRINT_RST
#define  GPIO_SW_RST_PIN_M_GPIO     GPIO_MODE_00

#define  GPIO_SW_RST_M_DAIPCMOUT    GPIO_MODE_01

//interrupt mode
#if MTK_L5_X_IRQ_SET
#define  GPIO_SW_EINT_PIN_M_GPIO    GPIO_FIGERPRINT_INT_M_GPIO //GPIO_MODE_00
#define  GPIO_SW_EINT_PIN_M_EINT    GPIO_FIGERPRINT_INT_M_EINT //GPIO_MODE_00
#endif

#endif
//-----------------------------------------------------------------------------
#include "sf_auto.h"

#endif
