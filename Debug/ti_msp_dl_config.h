/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for UART_UP */
#define UART_UP_INST                                                       UART2
#define UART_UP_INST_FREQUENCY                                           4000000
#define UART_UP_INST_IRQHandler                                 UART2_IRQHandler
#define UART_UP_INST_INT_IRQN                                     UART2_INT_IRQn
#define GPIO_UART_UP_RX_PORT                                               GPIOA
#define GPIO_UART_UP_TX_PORT                                               GPIOA
#define GPIO_UART_UP_RX_PIN                                       DL_GPIO_PIN_22
#define GPIO_UART_UP_TX_PIN                                       DL_GPIO_PIN_21
#define GPIO_UART_UP_IOMUX_RX                                    (IOMUX_PINCM47)
#define GPIO_UART_UP_IOMUX_TX                                    (IOMUX_PINCM46)
#define GPIO_UART_UP_IOMUX_RX_FUNC                     IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_UP_IOMUX_TX_FUNC                     IOMUX_PINCM46_PF_UART2_TX
#define UART_UP_BAUD_RATE                                               (115200)
#define UART_UP_IBRD_4_MHZ_115200_BAUD                                       (2)
#define UART_UP_FBRD_4_MHZ_115200_BAUD                                      (11)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Defines for ADC_line_detector */
#define ADC_line_detector_INST                                              ADC1
#define ADC_line_detector_INST_IRQHandler                         ADC1_IRQHandler
#define ADC_line_detector_INST_INT_IRQN                          (ADC1_INT_IRQn)
#define ADC_line_detector_ADCMEM_detector_out                      DL_ADC12_MEM_IDX_0
#define ADC_line_detector_ADCMEM_detector_out_REF         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_line_detector_ADCMEM_detector_out_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC_line_detector_C0_PORT                                     GPIOA
#define GPIO_ADC_line_detector_C0_PIN                             DL_GPIO_PIN_15
#define GPIO_ADC_line_detector_IOMUX_C0                          (IOMUX_PINCM37)
#define GPIO_ADC_line_detector_IOMUX_C0_FUNC      (IOMUX_PINCM37_PF_UNCONNECTED)



/* Defines for DMA_detector_out */
#define DMA_detector_out_CHAN_ID                                             (0)
#define ADC_line_detector_INST_DMA_TRIGGER            (DMA_ADC1_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for PIN_SCL: GPIOA.0 with pinCMx 1 on package pin 1 */
#define GPIO_OLED_PIN_SCL_PIN                                    (DL_GPIO_PIN_0)
#define GPIO_OLED_PIN_SCL_IOMUX                                   (IOMUX_PINCM1)
/* Defines for PIN_SDA: GPIOA.1 with pinCMx 2 on package pin 2 */
#define GPIO_OLED_PIN_SDA_PIN                                    (DL_GPIO_PIN_1)
#define GPIO_OLED_PIN_SDA_IOMUX                                   (IOMUX_PINCM2)
/* Port definition for Pin Group GPIO_KNOB */
#define GPIO_KNOB_PORT                                                   (GPIOB)

/* Defines for A: GPIOB.18 with pinCMx 44 on package pin 37 */
// pins affected by this interrupt request:["A"]
#define GPIO_KNOB_INT_IRQN                                      (GPIOB_INT_IRQn)
#define GPIO_KNOB_INT_IIDX                      (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_KNOB_A_IIDX                                    (DL_GPIO_IIDX_DIO18)
#define GPIO_KNOB_A_PIN                                         (DL_GPIO_PIN_18)
#define GPIO_KNOB_A_IOMUX                                        (IOMUX_PINCM44)
/* Defines for B: GPIOB.19 with pinCMx 45 on package pin 38 */
#define GPIO_KNOB_B_PIN                                         (DL_GPIO_PIN_19)
#define GPIO_KNOB_B_IOMUX                                        (IOMUX_PINCM45)
/* Defines for S: GPIOB.20 with pinCMx 48 on package pin 41 */
#define GPIO_KNOB_S_PIN                                         (DL_GPIO_PIN_20)
#define GPIO_KNOB_S_IOMUX                                        (IOMUX_PINCM48)
/* Port definition for Pin Group Gray_Address */
#define Gray_Address_PORT                                                (GPIOA)

/* Defines for AD0: GPIOA.24 with pinCMx 54 on package pin 44 */
#define Gray_Address_AD0_PIN                                    (DL_GPIO_PIN_24)
#define Gray_Address_AD0_IOMUX                                   (IOMUX_PINCM54)
/* Defines for AD1: GPIOA.25 with pinCMx 55 on package pin 45 */
#define Gray_Address_AD1_PIN                                    (DL_GPIO_PIN_25)
#define Gray_Address_AD1_IOMUX                                   (IOMUX_PINCM55)
/* Defines for AD2: GPIOA.26 with pinCMx 59 on package pin 46 */
#define Gray_Address_AD2_PIN                                    (DL_GPIO_PIN_26)
#define Gray_Address_AD2_IOMUX                                   (IOMUX_PINCM59)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_UP_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_ADC_line_detector_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
