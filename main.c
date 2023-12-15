/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSoC 4: CAPSENSE Low Power
 *              code example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/
 
/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "stdio.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#ifdef COMPONENT_PSOC4100SMAX
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#else
#define CAPSENSE_INTR_PRIORITY           (3u)
#endif

#define CY_ASSERT_FAILED                 (0u)

/* ILO Frequency in Hz */
#define ILO_FREQUENCY_HZ                 (40000U)

/* WDT interrupt period in milliseconds. Max limit is 1698 ms */
#define WDT_INTERRUPT_INTERVAL_MS        (10U)

/* WDT interrupt priority */
#define WDT_INTERRUPT_PRIORITY           (3u)

/* Define desired delay in microseconds */
#define DESIRED_WDT_INTERVAL             (WDT_INTERRUPT_INTERVAL_MS  * 1000U)

/* LED STATE TOGGLE MACROS */
#ifdef COMPONENT_PSOC4100SP
#define LED_ON    (1u)
#define LED_OFF   (0u)
#else
#define LED_ON    (0u)
#define LED_OFF   (1u)
#endif

/* EZI2C interrupt priority must be higher than CAPSENSE interrupt */
#if (defined(CAPSENSE_TUNER_ENABLE))
#define EZI2C_INTR_PRIORITY              (2u)
#endif

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/

/* WDT interrupt service routine configuration */
const cy_stc_sysint_t wdt_isr_cfg =
{
    .intrSrc = srss_interrupt_wdt_IRQn,    /* Interrupt source */
    .intrPriority = WDT_INTERRUPT_PRIORITY /* Interrupt priority is 3 */
};

/* Variable to check whether WDT interrupt is triggered */
volatile bool interrupt_flag = false;

/* Variable to store the counts required after ILO compensation */
static uint32_t ilo_compensated_counts = 0U;
static uint32_t ilo_cycles  = 0U;

cy_stc_scb_ezi2c_context_t ezi2c_context;

/* UART context structure */
cy_stc_scb_uart_context_t CYBSP_UART_context;

/* CAPSENSE Slider info */
cy_stc_capsense_touch_t *slider_touch_info;
uint16_t slider_pos = 0;
uint8_t slider_touch_status;
uint16_t prev = -1;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/* WDT interrupt service routine */
void wdt_isr(void);
/* WDT function */
void wdt_trigger(void);

cy_en_syspm_status_t deep_sleep_callback(
    cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode);

static void initialize_capsense(void);

#ifdef COMPONENT_PSOC4100SMAX
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
#else
static void capsense_isr(void);
#endif

#if (defined(CAPSENSE_TUNER_ENABLE))
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
#endif
static void control(void);
char uartString[50] = " ";

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 * Parameters:
 *  void
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable interrupt */
    cy_en_sysint_status_t sysintStatus = Cy_SysInt_Init(&wdt_isr_cfg, wdt_isr);
    if(sysintStatus != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_EnableIRQ(wdt_isr_cfg.intrSrc);

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Send a string over serial terminal */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n**************CE239069-PSoC 4:CAPSENSE Low Power**************\r\n");

    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* This function initializes the WDT block */
    Cy_WDT_Init();

    /* Enable the ILO */
    Cy_SysClk_IloEnable();

    /* Now switch the WDC timers clocking to ILO */
    /* Disable the WCO */
    Cy_SysClk_WcoDisable();

    /* Enable WDT */
    Cy_WDT_Enable();

    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* Initialize CapSense */
    initialize_capsense();

#if (defined(CAPSENSE_TUNER_ENABLE))
    /* Initialize EZI2C */
    initialize_capsense_tuner();

    cy_stc_syspm_callback_params_t ezi2cCallbackParams =
    {
        .base       = CYBSP_EZI2C_HW,
        .context    = &ezi2c_context
    };

    cy_stc_syspm_callback_t ezi2cCallback =
    {
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };
#endif

#ifdef COMPONENT_PSOC4100SMAX
    cy_stc_syspm_callback_params_t sysClkCallbackParams0 =
    {
        .base       = CYBSP_MSC0_HW,
        .context    = &cy_capsense_context
    };


    cy_stc_syspm_callback_params_t sysClkCallbackParams1 =
    {
        .base       = CYBSP_MSC1_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback0 =
    {
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams0,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };
    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback1 =
    {
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams1,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback0);
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback1);
#else
    cy_stc_syspm_callback_params_t sysClkCallbackParams =
    {
        .base       = CYBSP_CSD_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);
#endif

#if (defined(CAPSENSE_TUNER_ENABLE))
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
#endif

#ifdef COMPONENT_PSOC4100SMAX
    /* Start the first scan*/
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);
#else
    /* Start the first scan*/
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
#endif

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Turns LED ON/OFF and prints the Slider position based on Slider status */
            control();

            /* WDT interrupt source */
            wdt_trigger();

#ifdef COMPONENT_PSOC4100SMAX
            Cy_CapSense_ScanAllSlots(&cy_capsense_context);
#else
            /* Start the next scan*/
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
#endif

#if (defined(CAPSENSE_TUNER_ENABLE))
            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);
#endif
        }
    }
}

/*******************************************************************************
 * Function Name: wdt_trigger
 ********************************************************************************
 * Summary:
 *  - Updates the set match value to the WDT block.
 *  - Enters into deep sleep mode.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
void wdt_trigger(void)
{
    if (interrupt_flag)
    {
        /* Clear the interrupt flag */
        interrupt_flag = false;

        /* Update the match count  */
        Cy_WDT_SetMatch((uint16_t)(ilo_compensated_counts + Cy_WDT_GetMatch()));
    }

    /* Start ILO measurement */
    Cy_SysClk_IloStartMeasurement();

    /* Get the ILO compensated counts i.e. the actual counts for the
     desired ILO frequency. ILO default accuracy is +/- 60%.
     Note that DESIRED_WDT_INTERVAL should be less than the total
     count time */
    while (CY_SYSCLK_SUCCESS != Cy_SysClk_IloCompensate(DESIRED_WDT_INTERVAL, &ilo_cycles));
    ilo_compensated_counts = (uint32_t)ilo_cycles;
    /* Stop ILO measurement before entering deep sleep mode */
    Cy_SysClk_IloStopMeasurement();
    /* Enter deep sleep mode */
    Cy_SysPm_CpuEnterDeepSleep();
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense and configures the CapSense
 *  interrupt.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

#ifdef COMPONENT_PSOC4100SMAX
    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };
#else
    /* CapSense interrupt configuration */
    const cy_stc_sysint_t capsense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };
#endif

    /* Capture the CSD/MSC HW block and initialize it to the default state */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
#ifdef COMPONENT_PSOC4100SMAX
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);
#else
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);
#endif

        /* Initialize the CapSense firmware modules */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the README.md file
         */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*****************************************************************************
 * Function Name: wdt_isr
 ******************************************************************************
 * Summary:
 * This function is the handler for the WDT interrupt
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************************/
void wdt_isr(void)
{
    /* Clears the WDT match flag */
    Cy_WDT_ClearInterrupt();
    /* Set the interrupt flag */
    interrupt_flag = true;
}

#if (defined(CAPSENSE_TUNER_ENABLE))
/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CapSense Tuner tool.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

}

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}
#endif

#ifdef COMPONENT_PSOC4100SMAX
/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: capsense_msc1_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC1 block.
 *
 *******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}
#else
/*******************************************************************************
 * Function Name: capsense_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense block.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}
#endif

/*******************************************************************************
* Function Name: control
********************************************************************************
* Summary:
* Turning LED ON/OFF and prints the Slider position based on Slider status
*
*******************************************************************************/
static void control(void)
{
    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;

    if (slider_touch_status != 0)
    {
        slider_pos = slider_touch_info->ptrPosition->x;
    }

    if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
        if (slider_touch_status != 0)
        {
            if (prev  != slider_pos)
            {
                sprintf(uartString, "Slider position = %ld\r\n",(unsigned long)slider_pos);
                prev = slider_pos;
                Cy_SCB_UART_PutString(CYBSP_UART_HW,uartString);
            }
        }
    }

    else if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
    }
#ifdef COMPONENT_PSOC4100SMAX
    else if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
    }
#endif

#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP

    else if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
    }

    else if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON2_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
    }
#endif
    else
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_OFF);
    }
}

/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 * Summary:
 * Deep Sleep callback implementation. It changes UART status based on
 * the power state and MCU state.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure
 *  cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
    cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {    /* Check if the device is ready to enter the low power mode */
    case CY_SYSPM_CHECK_READY:

        while(Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW) == 0U)
        {
            /* Wait until the TX FIFO
             * and Shifter are empty and there is no more data to send. */
        }

        /* Disable the UART */
        Cy_SCB_UART_Disable(CYBSP_UART_HW, &CYBSP_UART_context);

        ret_val = CY_SYSPM_SUCCESS;
        break;

        /* Roll back the actions performed in the previously executed callback with CY_SYSPM_CHECK_READY */
    case CY_SYSPM_CHECK_FAIL:

        /* Enable the UART */
        Cy_SCB_UART_Enable(CYBSP_UART_HW);

        ret_val = CY_SYSPM_SUCCESS;
        break;

        /* Performs the actions to be done after exiting the low power mode if entered */
    case CY_SYSPM_AFTER_TRANSITION:

        /* Enable the UART */
        Cy_SCB_UART_Enable(CYBSP_UART_HW);

        ret_val = CY_SYSPM_SUCCESS;
        break;

    default:
        /* Don't do anything in the other modes */
        ret_val = CY_SYSPM_SUCCESS;
        break;
    }
    return ret_val;
}

/* [] END OF FILE */