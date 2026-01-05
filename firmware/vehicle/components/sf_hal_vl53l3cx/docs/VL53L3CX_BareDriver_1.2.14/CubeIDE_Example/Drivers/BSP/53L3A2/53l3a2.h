/**
 * @file 53L3A2.h
 *
 *  Created on: March 24, 2020
 *      Author: ST Imaging
 */

#ifndef _X_NUCLEO_53L3A2_H_
#define _X_NUCLEO_53L3A2_H_

#include  "stm32xxx_hal.h"

/**
 * @defgroup VL53L3A2_config  VL53L3A2 static configuration
 * @brief Configure BSP support for USART2 (nucleo virtual com)
 * UART support implies that BSP exclusively manages:
 *  @li HAL_UART_MspInit and HAL_UART_MspDeInit
 *  @li DMA channel and interrupt
 */


/** @ingroup VL53L3A2_config
 * @{*/


#ifndef VL53L3A2_HAVE_UART
/**
 * Define this macro at project level if UART is available
 */
#	define VL53L3A2_HAVE_UART 0
#endif


#if VL53L3A2_HAVE_UART

/**
 * User override of baud rate programmed for uart2 (default to 115200)
 */
#ifndef USART2_BAUD_RATE
#   define USART2_BAUD_RATE    115200
#endif


#ifndef VL53L3A2_UART_DMA_RX
/** configure use of dma for uart rx
 *
 * Default to not use DMA RX when not defined explicitly
 *
 * require @a VL53L3A2_HAVE_UART
 */
#   define VL53L3A2_UART_DMA_RX   0
#endif //VL53L3A2_UART_DMA_RX

#ifndef VL53L3A2_UART_DMA_TX
/** configure use of dma for uart tx
 *
 * Default to use DMA TX when not defined explicitly by user
 *
 * require @a VL53L3A2_HAVE_UART
 */
#   define VL53L3A2_UART_DMA_TX   1
#endif //VL53L3A2_UART_DMA_RX


#ifndef  VL53L3A2_UART_DMA_RX_IRQ_PRI
/** configure usart  max irq priority
 *
 * Default to 0 when not defined explicitly by user
 * @warning may not be support by all mcu (only F401 tested)
 */
#   define  VL53L3A2_UART_DMA_RX_IRQ_PRI   0
#endif

#ifndef  VL53L3A2_UART_DMA_TX_IRQ_PRI
/** configure usart dma tx irq priority
 *
 * Default to 0 when not defined explicitly
 * @warning only F401
 */

#   define  VL53L3A2_UART_DMA_TX_IRQ_PRI   0
#endif

#ifndef VL53L3A2_UART_IRQ_PRI
/**
 * User override default uart irq priority 0 to fit application needs
 * @warning only supported for f401
 */
#   define VL53L3A2_UART_IRQ_PRI    0
#endif

/* provided by MSP part */
extern void XNUCLEO53L3A2_USART2_UART_Init(void);

#else
#	define XNUCLEO53L3A2_USART2_UART_Init(...) (void)0
#endif

/** @}  */

/**
 * Configure interrupt pins pins pull up/down
 *
 * set to GPIO_NOPULL or GPIO_PULLUP or GPIO_PULLDOWN
 */
#define VL53L3A2_INTR_PIN_PUPD GPIO_PULLUP



/**
 * @ingroup VL53L3A2_GPIO1_MAP
 * @{
 */

#ifndef VL53L3A2_GPIO1_SHARED
/**
 * @brief select use of shared interrupt
 *
 * Must be set to 0 (or not defined) for one interrupt per sensor
 * Must be set to non 0 for shared interrupt line
 * see @sa VL53L3A2_GPIO1_C_OPTION
 */
#   define VL53L3A2_GPIO1_SHARED  0
#endif


#ifndef VL53L3A2_GPIO1_C_OPTION
/**
 * @def VL53L3A2_GPIO1_C_OPTION
 * @brief select GPIO1 to exti mapping for center or shared interrupt
 *
 * Set option value or un-define it to match with board configuration
 * @li not defined or 0  : U14=On  and U17=off  => GPIO1_C = PA4
 * @li defined and not 0 : U14=Off and U17=on  => GPIO1_C = PC1
 */
#   define VL53L3A2_GPIO1_C_OPTION 0
#endif

#ifndef VL53L3A2_GPIO1_L_OPTION
/**
 * @def VL53L3A2_GPIO1_L_OPTION
 * @brief control left sensor GPIO1 routing solder option
 * @warning not used on shared interrupt config
 *
 * Set option value or un-define it to match with board configuration
 * @li not defined or 0  : U10=on and  U11=Off => GPIO1_L = PC7
 * @li defined and not 0 : U10=off and U11=on  => GPIO1_L = PA9
 */
#   define VL53L3A2_GPIO1_L_OPTION 0
#endif

#ifndef VL53L3A2_GPIO1_R_OPTION
/**
 * @def VL53L3A2_GPIO1_R_OPTION
 * @brief control right sensor GPIO1 routing solder option
 * @warning not used on shared interrupt config
 *
 * Set option value or un-define it to match with board configuration
 * @li not defined or 0  : U18=on and  U15=Off => GPIO1_R = PA10
 * @li defined and not 0 : U18=off and U15=on  => GPIO1_R = PB5
 */
#   define VL53L3A2_GPIO1_R_OPTION 0
#endif


#if VL53L3A2_GPIO1_C_OPTION == 0
#   define VL53L3A2_GPIO1_C_GPIO_PORT   GPIOA
#   define VL53L3A2_GPIO1_C_CLK_ENABLE  __GPIOA_CLK_ENABLE
#   define VL53L3A2_GPIO1_C_GPIO_PIN    GPIO_PIN_4
#   if defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32F401xC)
#       define VL53L3A2_GPIO1_C_INTx       EXTI4_IRQn
#       define VL53L3A2_EXTI4_USE_PIN      VL53L3A2_GPIO1_C_GPIO_PIN
#   endif
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_C_INTx        EXTI4_15_IRQn
#       define VL53L3A2_EXTI4_15_USE_PIN    VL53L3A2_GPIO1_C_GPIO_PIN
#   endif // STM32L053xx
#else
#   define VL53L3A2_GPIO1_C_CLK_ENABLE  __GPIOC_CLK_ENABLE
#   define VL53L3A2_GPIO1_C_GPIO_PORT   GPIOC
#   define VL53L3A2_GPIO1_C_GPIO_PIN    GPIO_PIN_1
#   if defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32F401xC)
#       define VL53L3A2_GPIO1_C_INTx        EXTI1_IRQn
#       define VL53L3A2_EXTI1_USE_PIN       VL53L3A2_GPIO1_C_GPIO_PIN
#   endif // STM32F401xE
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_C_INTx        EXTI0_1_IRQn
#       define VL53L3A2_EXTI0_1_USE_PIN     VL53L3A2_GPIO1_C_GPIO_PIN
#   endif // STM32L053xx
#endif

#if VL53L3A2_GPIO1_SHARED == 0
/* not shared interrupt configuration */
#if VL53L3A2_GPIO1_L_OPTION == 0
#   define VL53L3A2_GPIO1_L_GPIO_PORT   GPIOC
#   define VL53L3A2_GPIO1_L_CLK_ENABLE  __GPIOC_CLK_ENABLE
#   define VL53L3A2_GPIO1_L_GPIO_PIN    GPIO_PIN_7
#   if defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32F401xC)
#       define VL53L3A2_GPIO1_L_INTx       EXTI9_5_IRQn
#       define VL53L3A2_EXTI9_5_USE_PIN    VL53L3A2_GPIO1_L_GPIO_PIN
#   endif // STM32F401xE
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_L_INTx        EXTI4_15_IRQn
#       define VL53L3A2_EXTI4_15_USE_PIN_2   VL53L3A2_GPIO1_L_GPIO_PIN
#       ifndef VL53L3A2_EXTI4_15_USE_PIN
#           define VL53L3A2_EXTI4_15_USE_PIN    VL53L3A2_GPIO1_L_GPIO_PIN
#       else
#           warning "shared use of interrupt 7 in group  4-15 for L sensor "
#       endif
#   endif // STM32L053xx
#else
#   define VL53L3A2_GPIO1_L_GPIO_PORT   GPIOA
#   define VL53L3A2_GPIO1_L_CLK_ENABLE  __GPIOA_CLK_ENABLE
#   define VL53L3A2_GPIO1_L_GPIO_PIN    GPIO_PIN_9
#   if defined(STM32F401xE) || defined(STM32L476xx)
#       define VL53L3A2_GPIO1_L_INTx       EXTI9_5_IRQn
#       define VL53L3A2_EXTI9_5_USE_PIN    VL53L3A2_GPIO1_L_GPIO_PIN
#   endif // STM32F401xE
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_L_INTx        EXTI4_15_IRQn
#       define VL53L3A2_EXTI4_15_USE_PIN_2    VL53L3A2_GPIO1_L_GPIO_PIN
#       ifndef VL53L3A2_EXTI4_15_USE_PIN
#           define VL53L3A2_EXTI4_15_USE_PIN    VL53L3A2_GPIO1_L_GPIO_PIN
#       else
#           #warning "shared use of interrupt 9 in group 4-15 for L sensor"
#       endif
#   endif // STM32L053xx
#endif //else VL53L3A2_GPIO1_L_OPTION

#if VL53L3A2_GPIO1_R_OPTION == 0
#   define VL53L3A2_GPIO1_R_GPIO_PORT   GPIOA
#   define VL53L3A2_GPIO1_R_CLK_ENABLE  __GPIOA_CLK_ENABLE
#   define VL53L3A2_GPIO1_R_GPIO_PIN    GPIO_PIN_10
#   if defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32F401xC)
#       define VL53L3A2_GPIO1_R_INTx       EXTI15_10_IRQn
#       define VL53L3A2_EXTI15_10_USE_PIN  VL53L3A2_GPIO1_R_GPIO_PIN
#   endif // STM32F401xE
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_R_INTx        EXTI4_15_IRQn
#       define VL53L3A2_EXTI4_15_USE_PIN_3  VL53L3A2_GPIO1_R_GPIO_PIN
#       ifndef VL53L3A2_EXTI4_15_USE_PIN
#           define VL53L3A2_EXTI4_15_USE_PIN    VL53L3A2_GPIO1_R_GPIO_PIN
#       else
#           warning "shared use of interrupt 10 in group 4-15 for R sensor"
#       endif
#   endif // STM32L053xx
#else
#   define VL53L3A2_GPIO1_R_GPIO_PORT   GPIOB
#   define VL53L3A2_GPIO1_R_CLK_ENABLE  __GPIOB_CLK_ENABLE
#   define VL53L3A2_GPIO1_R_GPIO_PIN    GPIO_PIN_5
#   if defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32F401xC)
#       define VL53L3A2_GPIO1_R_INTx       EXTI9_5_IRQn //  conflict with VL53L3A2_GPIO1_L any option
#       define VL53L3A2_EXTI9_5_USE_PIN    VL53L3A2_GPIO1_R_GPIO_PIN
#   endif // STM32F401xE
#   if defined(STM32L053xx)
#       define VL53L3A2_GPIO1_R_INTx        EXTI4_15_IRQn
#       define VL53L3A2_EXTI4_15_USE_PIN_3  VL53L3A2_GPIO1_R_GPIO_PIN
#       ifndef VL53L3A2_EXTI4_15_USE_PIN
#           define VL53L3A2_EXTI4_15_USE_PIN    VL53L3A2_GPIO1_R_GPIO_PIN
#       else
#           #warning "shared use of interrupt 5 in group 4-15 for R sensor"
#       endif // STM32L053xx
#   endif // STM32L053xx
#endif //else VL53L3A2_GPIO1_R_OPTION

#endif //else VL53L3A2_GPIO1_SHARED

/** @} */ /* defgroup L53L3A2_GPIO1_MAP */

/**
 * @defgroup XNUCLEO53L3A2_Debugging VL53L3A2  debugging
 * @{
 */

#ifndef  XNUCLEO53L3A2_TRACE
/**
 * @brief enable error output via trace
 *
 * when undefined (default) no trace no error logging is done, it is safe
 * to at least count error just to see if any errors ever occur.
 *
 * Traces formating and output is end user defined via #trace_printf
 */
#   define XNUCLEO53L3A2_TRACE    TRACE_UART
#endif // XNUCLEO53L3A2_TRACE


/**
 * @def XNUCLEO53L3A2_ErrLog(...)
 * Macro used to report error log messages with printf format
 *
 * Our testing version use externally trace_printf,
 * We trace out function names and line numbers plus any text formated with some extra arguments
 */
#if XNUCLEO53L3A2_TRACE
#   define XNUCLEO53L3A2_ErrLog( msg, ...) trace_printf("[Err] %s l %d \t" msg "\n", __func__, __LINE__, ##__VA_ARGS__)
#else
#   define XNUCLEO53L3A2_ErrLog(...) (void)0
#endif



/** @} */ /* group XNUCLEO53L3A2_Debugging */


/**
 * @defgroup XNUCLEO53L3A2_Interface X-NUCLEO-53L3A2 BSP API
 * @{ */

/**
 * 53L0X Device selector
 *
 * @note Most functions are using a device selector as input. ASCII 'c', 'l' or 'r' are also accepted.
 */
enum XNUCLEO53L3A2_dev_e{
    XNUCLEO53L3A2_DEV_LEFT =  0,    //!< left satellite device P21 header : 'l'
    XNUCLEO53L3A2_DEV_CENTER  =  1, //!< center (built-in) vl053 device : 'c"
    XNUCLEO53L3A2_DEV_RIGHT=  2     //!< Right satellite device P22 header : 'r'
};

/**
 * I2C1 handle
 * @note setup and configured by @ref XNUCLEO53L3A2_Init
 */
extern I2C_HandleTypeDef  XNUCLEO53L3A2_hi2c;
/** UART2 handle
 *
 * UART2 is the nucleo Virtual Com Port
 * @note setup and configured by @ref  XNUCLEO53L3A2_Init*/
#if VL53L3A2_HAVE_UART
extern UART_HandleTypeDef huart2;

#if VL53L3A2_UART_DMA_RX
/** UART2 DMA RX available only if support activated*/
extern DMA_HandleTypeDef hdma_usart2_rx;
#endif

#if VL53L3A2_UART_DMA_TX
/** UART2 DMA TX available only if support activated */
extern DMA_HandleTypeDef hdma_usart2_tx;
#endif
#endif//ifdef VL53L3A2_HAVE_UART

/**
 * Initialize VL053L3A2 STM32 expansion board
 *
 * @note All VL53L0X devices XSDN are asserted and display is turned off
 * @return 0 on success
 */
int XNUCLEO53L3A2_Init(void);

/**
 * Set Reset (XSDN) state of a given "id" device
 * @param  DevNo The device number use  @ref XNUCLEO53L3A2_dev_e. Char 't' 'c' 'r' can also be used
 * @param  state  State of the device reset (xsdn) pin @warning reset  pin is active low
 * @return 0 on success
 */
int XNUCLEO53L3A2_ResetId(int DevNo,  int state );

/**
 * Get PB1 push button state
 *
 * @param state  Actual button state 0/1 boolean type
 * @warning In case of error, value of state is unchanged
 * @return 0 on success
 */
int XNUCLEO53L3A2_GetPB1(int *state);

/**
 * Set the 7 segments display
 * @param str  String to set on display
 * @warning When string is less than 4 digits, display is left-justified and lower digits are blanked.
 *          To display a 2 digits value, left justified on the 4 digits, use "%4d" formating
 * @warning When more than 4 char are present only first 4 are displayed
 * @note    Characters that do not have 7 segment font matching in @ref ascii_to_display_lut are left blank
 * @return 0 on success
 */
int XNUCLEO53L3A2_SetDisplayString(const char *str);
/** @} */ /* defgroup XNUCLEO53L3A2_Interface */

/**
 * @ingroup XNUCLEO53L3A2_Interface
 * @defgroup MSP_implement MSP and User specific code
 *
 * Some MSP Micro-controller Support Package code is provided in the BSP source.\n
 * The following MSP function and callback are already implemented
 * ### UART ###
 * @li HAL_UART_MspDeInit and HAL_UART_MspInit
 * @li HAL_UART_TxCpltCallback
 * @li USART2_IRQHandler
 *
 * ### EXTI ###
 *   @li F401RE/L476 EXTI15_10_IRQHandler, EXTI9_5_IRQHandler, EXTI4_IRQHandler and EXTI1_IRQHandler
 *   @li L053R8 EXTI4_15_IRQHandler, EXTI0_1_IRQHandler and EXTI4_15_IRQHandler
 *   @li HAL_GPIO_EXTI_Callback
 *
 * ### DMA ###
 *   @li L476 DMA1_Channel6_IRQHandler and DMA1_Channel7_IRQHandler
 *   @li F401 DMA1_Stream5_IRQHandler and DMA1_Stream6_IRQHandler
 *   @li L053 DMA1_Channel4_5_6_7_IRQHandler
 *
 * Exact EXTI list depends on soldering options and configured DMA list depends on DMA RX/TX configuration options.
 *
 * If those functions and callbacks are also to be used by application, the code must be removed or adapted to manage multi-instances
 *
 * Family-specific code is on dedicated files:
 * @li vl53l0A1-l053msp.c for Nucleo-L053R8
 * @li vl53l0A1-x4msp.c for Nucleo-F401RE and Nucleo-L476RG
 * @li uart_trace.c for generic uart handling
 * @{
 */

/**
 * @defgroup MSP_implement_common MSP Common to all STM32
 *
 * MSP adaption that apply to all mcu (F4,L4,L0) with no or little customization.
 *
 * @{
 */

/**
 * Enable Disable interrupt at MCU level (MSP)
 *
 * Enable/Disable the interrupt for a given sensor Dev number
 *
 * End user is expected to override VL53L3A2_EXTI_Callback to catch interrupts.
 * When the vector is shared applicaton is passed the device and pin corretcy
 * In shared line mode application is responsible to find what sensor cause the interrupt and deal with races on interrupt line
 *
 * @param EnableIntr  "boolean" 0 to disable interrupt otherwise interrupt is enabled
 * @param DevNo       Device number "name"  of the sensor see @ref XNUCLEO53L3A2_dev_e
 *
 * @warning In shared interrupt mode, use only center : other values are not supported and will return an error
 * @warning If several lines share the same interrupt vector (l053 mcu), only one sensor interrupt shall be enable at a time
 *          otherwise the handler may fail to locate and clear the right source (no sharing management)
 *
 * @return  0 on success \n
 *         <0 for error (invalid id)
 *         >0 interrupt configured but with potential sharing on EXTI groups see @ref VL53L3A2_GPIO1_MAP
 */
int XNUCLEO53L3A2_SetIntrStateId(int EnableIntr, int DevNo);

/** @} */ /* MSP_implemant_common */

/**
 * MSP Configure expansion board Interrupt i/o
 *
 * This function configures GPIO i/o and NVIC.  See  @ref VL53L3A2_GPIO1_MAP for details
 * To enable interrupt handling use @ref XNUCLEO53L3A2_SetIntrStateId()
 *
 * The input pullup/pulldown is configured by static configuration @a #VL53L3A2_INTR_PIN_PUPD
 *
 * @warning  Interrupt is disabled and any pending status flushed out\n
 *       if initial vl53lx sensor interrupt state is important it must be handled externally (soft trigger)
 * @warning  If interrupt vector is shared it is better to configure and enable only one at a time
 *  proper shared management is done but yet some isr could be lost or some other fired spuriously.
 *
 * @param DevNo       device number see @ref XNUCLEO53L3A2_dev_e
 * @param IntPriority Interrupt Priority to set
 * @param SubPriority Interrupt SubPriority to set
 */
void VL53L3A2_EXTI_IOConfigure(int DevNo, int  IntPriority, int SubPriority);

/**
 * Un-configure sensor interrupt line to mcu
 *
 * The line is DeInited (put back to input?)
 *
 * This function disables the related i/o pin and flush line interrupt pending (not vector)
 * this is to permit shared vector to work.
 * To turn off vector use @ref XNUCLEO53L3A2_SetIntrStateId()
 *
 * @warning because vector and line disable are not done atomically with isr disable,
 *  a last invalid interrupt may be fired.
 *
 * @param DevNo device  see @ref XNUCLEO53L3A2_dev_e
 */
void VL53L3A2_EXTI_IOUnconfigure(int DevNo);

/** @} */ /* MSP_implement */


#endif /* _X_NUCLEO_53L3A2_H_ */
