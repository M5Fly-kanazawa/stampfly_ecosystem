/**
 * @file 53L3A2.c
 *
 * implement X-NUCLEO-53L3A2 Nucleo BSP
 */


#include <string.h>
#include  "53L3A2.h"

#include "stm32xxx_hal.h"


#ifndef HAL_I2C_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#pragma message("hal conf should enable i2c")
#endif

/* when not customized by application define dummy one */
#ifndef XNUCLEO53L3A2_GetI2cBus
/**
 * macro that can be overloaded by user to enforce i2c sharing in RTOS context
 */
#define XNUCLEO53L3A2_GetI2cBus(...) (void)0
#endif

#ifndef XNUCLEO53L3A2_PutI2cBus
/** macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define XNUCLEO53L3A2_PutI2cBus(...) (void)0
#endif

/**
 *@mainpage
 *@image html VL53L3A2_board.jpg "X-NUCLEO-53L3A2 Expansion Board"

  @section Quick BSP links
  - @ref XNUCLEO53L3A2_Interface
  - Board information and configuration @ref XNUCLEO53L3A2_Board
  - Go to Modules panes


  @section XNUCLEO53L3A2_usage How to use X-NUCLEO-53L3A2 BSP ?
  ## Multiple family project ##
   X-NUCLEO-53L3A2 BSP supports multiple STM32 families through unique "stm32xxx_hal.h".
   This file redirects to the targeted stm32 HAL.
   @note This file can also be used to place any customization defines

  @section XNUCLEO53L3A2_conf  How to configure X-NUCLEO-53L3A2 BSP ?
  ## configuration ##
    see @ref VL53L3A2_config

  ### MSP implementation ###
   @li @ref MSP_implement
     @li @ref MSP_implement_common
     @li @ref MSP_implement_STM32x4

*/


/**
 * @defgroup XNUCLEO53L3A2_Board X-NUCLEO-53L3A2 Expansion Board
 * @{
 */

/**
 * @defgroup VL53L3A2_GPIO1_MAP    VL53L3A2 Sensor to MCU interrupt mapping
 *
 * # GPIO mapping ##
 * Various options can be used to map the 3 VL53L0X interrupt lines to MCU. By default, the expansion board is configured in
 * Center on-board vl53l0x mode which means only the center device can be used in interrupt mode. To use left and/or right devices
 * in interrupt mode, it is necessary to adapt the board configuration as described below.
 * ## One interrupt line shared by all sensors ##
 * All VL53L0x GPIO1 pins (open collector outputs) are connected together on the expansion board
 * and level shifter input drives single shared interrupt line to MCU.\n
 * Solder options to operate in this mode:
 * @li U7 and U8 are fitted  (connect GPIO all together before level shifter)
 * @li U10, U11, U15 and U18 are not soldered. (disconnect all level shifted option to arduino connector)
 * @li U14/U17 to select  PA4/PC1  => EXTI4_15_IRQn / EXTI0_1_IRQn (final selection option)
 *
 * see @a #VL53L3A2_GPIO1_C_OPTION for interrupt line selection
 *
 * @note requires @a #VL53L3A2_GPIO1_SHARED to be set (to 1)
 *
 * ##  One interrupt per sensor  ##
 * To use one interrupt per device :\n
 * @li Do not define @a #VL53L3A2_GPIO1_SHARED  or set to 0
 * @li U7 and U8 must not be soldered (disconnect L and R GPIO from C before level shifter)
 *
 * ### Center on-board vl53l0x  ###
 * @li U14 (fitted by default)  CN8#3    PA4
 * @li U17 (*option)         	CN8#5    PC1
 *
 * see @ref VL53L3A2_GPIO1_C_OPTION
 *
 * ### Left satellite ###
 * @li U10 (not fitted by default)  CN5#2    PC7
 * @li U11 (*option)         		CN5#1    PA9
 *
 * see @a #VL53L3A2_GPIO1_L_OPTION
 *
 * ### Right satellite ###
 * @li U18 (not fitted by default)  CN9#3    PA10
 * @li U15 (*option)       			CN9#5    PB5

 *
 * see @a #VL53L3A2_GPIO1_R_OPTION
 *
 * ### Interrupt vectors F401 and L476 ###
 * @li Center PA4 / PC1  => Extit4 / Exti 1
 * @li Left PC7 / PA9 => Extit9_5 / Extit9_5
 * @li Right PA10 / PB5 => Extit15_10 / Extit9_5
 *
 * @warning When selecting alternate option for both sensor L/R interrupt line will shared same vector
 *
 * ### L053 ###
 * @li Center PA4 / PC1  => EXTI4_15_IRQn / EXTI0_1_IRQn
 * @li Left PC7 / PA9  => EXTI4_15_IRQn / EXTI4_15_IRQn
 * @li Right PA10 /  PB5 => EXTI4_15_IRQn / EXTI4_15_IRQn
 *
 * @warning R and L have shared vector use, and will also share C vector if alternate option is selected for C
 */

/**
 * @defgroup  XNUCLEO53L3A2_I2CExpanders I2C expender mapping
 * ## I2C Expanders address and i/o ##
 * Digit 1 2 3 and 4 below are numbered left to right "1234" on see on display
 * ### Expander 0 ###
 * U21 A[2-0]= 001 => i2c address[7..1] 0x43 ([7..0] 0x86 )
 * @li Digit#1  gpio 0 to 6
 * @li Digit#2  gpio 7 to 13
 * @li xshut_l  gpio 14
 * @li xshut_r  gpio 15
 *
 * ### Expander 1 ###
 * U19 A[2-0]= 000 => i2c address[7..1] 0x42 ([7..0] 0x84 )
 * @li Digit#3  gpio 0 to 6
 * @li Digit#4  gpio 7 to 13
 * @li PB1      gpio 14
 * @li xshut    gpio 15
 *
 * @note The 0/1  assignment is "digit" order logical don't look for any sense as  per device address or part number
 * @{
 */

/**
 * Expander 0 i2c address[7..0] format
 */
#define I2cExpAddr0 ((int)(0x43*2))
/**
 * Expander 1 i2c address[7..0] format
 */
#define I2cExpAddr1 ((int)(0x42*2))
/** @} XNUCLEO53L3A2_I2CExpanders*/


/**
 * GPIO monitor pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPMR    0x10
/**
 * STMPE1600 GPIO set pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPSR    0x12
/**
 * STMPE1600 GPIO set pin direction register
 * 16 bit register LSB at lowest offset
 */
#define GPDR    0x14


/** @} */ /* defgroup  XNUCLEO53L3A2_Board */


/****************************************************
 *@defgroup  XNUCLEO53L3A2_globals
 *@{
 */

/**
 *  i2c handle to be  use of all i2c access
 * end user shall provide it to
 * can be @a XNUCLEO53L3A2_I2C1Configure() @sa XNUCLEO53L3A2_usage
 * @warning do not use any XNUCLEO53L3A2_xxx prior to a first init with valid i2c handle
 */
I2C_HandleTypeDef  XNUCLEO53L3A2_hi2c;


/**
 * cache the full set of expanded GPIO values to avoid i2c reading
 */
static union CurIOVal_u {
    uint8_t bytes[4];   /*!<  4 bytes array i/o view */
    uint32_t u32;       /*!<  single dword i/o view */
}
/** cache the extended IO values */
CurIOVal;

/**
 * lookup table for for digit  to bit position in @a CurIOVal u32
 */
static int  DisplayBitPos[4]={0, 7, 16, 16+7};

/** @} XNUCLEO53L3A2_globals*/

/* Forward definition of private function */

static int _ExpanderRd(int I2cExpAddr, int index, uint8_t *data, int n_data);
static int _ExpanderWR(int I2cExpAddr, int index, uint8_t *data, int n_data);
static int _ExpandersSetAllIO(void);

/**
 * Expansion board i2c bus recovery
 *
 * We may get reset in middle of an i2c access (h/w reset button, debug or f/w load)
 * hence some agent on bus may be in middle of a transaction and can create issue or even prevent starting (SDA is low)
 * this routine does use gpio to manipulate and recover i2c bus line in all cases.
 */
static void _I2cFailRecover(){
    GPIO_InitTypeDef GPIO_InitStruct;
    int i, nRetry=0;


    // We can't assume bus state based on SDA and SCL state (we may be in a data or NAK bit so SCL=SDA=1)
    // by setting SDA high and toggling SCL at least 10 time we ensure whatever agent and state
    // all agent should end up seeing a "stop" and bus get back to an known idle i2c  bus state

    // Enable I/O
    __GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9 ;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //TODO we could do this faster by not using HAL delay 1ms for clk timing
    do{
        for( i=0; i<10; i++){
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_Delay(1);
        }
//        if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 ){
//            static int RetryRecover;
//            RetryRecover++;
//        }
    }while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 && nRetry++<7);

    if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 ){
        __GPIOA_CLK_ENABLE();
        //We are still in bad i2c state warm user by blinking led but stay here
        GPIO_InitStruct.Pin = GPIO_PIN_5 ;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        do{
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(33);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(33);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(33);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(33*20);
        }while(1);
    }
}



int XNUCLEO53L3A2_I2C1Configure() {
    int status;
    GPIO_InitTypeDef GPIO_InitStruct;

    _I2cFailRecover();

    /* Peripheral clock enable */
    __GPIOB_CLK_ENABLE();
    __I2C1_CLK_ENABLE();

    /**I2C1 GPIO Configuration\n
     PB8     ------> I2C1_SCL\n
     PB9     ------> I2C1_SDA
     */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    XNUCLEO53L3A2_hi2c.Instance = I2C1;
#ifdef __STM32F4xx_HAL_H
    XNUCLEO53L3A2_hi2c.Init.ClockSpeed = 400000;
    XNUCLEO53L3A2_hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
#else
    /* STM32L4xx and L053 */
    XNUCLEO53L3A2_hi2c.Init.Timing = 0x00300F38; /* set 400KHz fast mode i2c*/
#endif
    XNUCLEO53L3A2_hi2c.Init.OwnAddress1 = 0;
    XNUCLEO53L3A2_hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    XNUCLEO53L3A2_hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    XNUCLEO53L3A2_hi2c.Init.OwnAddress2 = 0;
    XNUCLEO53L3A2_hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    XNUCLEO53L3A2_hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    status = HAL_I2C_Init(&XNUCLEO53L3A2_hi2c);
    return status;
}

int XNUCLEO53L3A2_SetIntrStateId(int EnableIntr, int DevNo){
    int status;
    IRQn_Type IntrNo;
    int IntrPin;


    switch( DevNo ){
    case XNUCLEO53L3A2_DEV_CENTER :
    case 'c' :
        IntrNo = VL53L3A2_GPIO1_C_INTx;
        IntrPin= VL53L3A2_GPIO1_C_GPIO_PIN;
        status = 0;
        break;
#if ! VL53L3A2_GPIO1_SHARED
    case XNUCLEO53L3A2_DEV_LEFT :
    case 'l' :
        IntrNo = VL53L3A2_GPIO1_L_INTx;
        IntrPin= VL53L3A2_GPIO1_L_GPIO_PIN;
        if( VL53L3A2_GPIO1_L_INTx == VL53L3A2_GPIO1_R_INTx || VL53L3A2_GPIO1_L_INTx == VL53L3A2_GPIO1_C_INTx){
            XNUCLEO53L3A2_ErrLog("Conflicting Exti for %d",DevNo);
            status = 1;
            // for treating as error un-comment
            //goto done;
        }
        else{
            status =0;
        }
        break;
    case 'r' :
    case XNUCLEO53L3A2_DEV_RIGHT :
        IntrNo = VL53L3A2_GPIO1_R_INTx;
        IntrPin= VL53L3A2_GPIO1_R_GPIO_PIN;
        if( VL53L3A2_GPIO1_L_INTx == VL53L3A2_GPIO1_R_INTx || VL53L3A2_GPIO1_R_INTx == VL53L3A2_GPIO1_C_INTx  ){
            XNUCLEO53L3A2_ErrLog("Conflicting Exti for %d",DevNo);
            status = 1;
            // for treating as error un-comment
            //goto done;
        }
        else{
            status =0;
        }

        break;
#endif
    default:
        XNUCLEO53L3A2_ErrLog("Invalid DevNo %d",DevNo);
        status = -1;
        goto done;
    }

    if( EnableIntr ){
        __HAL_GPIO_EXTI_CLEAR_IT(IntrPin);
        NVIC_ClearPendingIRQ(IntrNo);
        HAL_NVIC_EnableIRQ(IntrNo);
        /**
         * @note  When enabling interrupt end user shall check actual state of the line and soft trigger event if active
         * Alternatively user can use API and device feature to clear device Interrupt status to possibly generates a new edge.
         * on shared pin configuration this must be repeated for all device.
         * The same shall be done after clearing a condition in device and interrupt remain active.
         */
    }
    else{
        HAL_NVIC_DisableIRQ(IntrNo);
        __HAL_GPIO_EXTI_CLEAR_IT(IntrPin);
        NVIC_ClearPendingIRQ(IntrNo);
    }

done:
    return status;
}


int XNUCLEO53L3A2_Init(void) {
    int status;
    uint8_t ExpanderData[2];
    XNUCLEO53L3A2_USART2_UART_Init();
    XNUCLEO53L3A2_I2C1Configure();

    status = _ExpanderRd( I2cExpAddr0, 0, ExpanderData, 2);
    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
        XNUCLEO53L3A2_ErrLog("I2C Expander @0x%02X not detected",(int)I2cExpAddr0 );
        goto done_err;

    }
    status = _ExpanderRd( I2cExpAddr1, 0, ExpanderData, 2);
    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
        XNUCLEO53L3A2_ErrLog("I2C Expander @0x%02X not detected",(int)I2cExpAddr1);
        goto done_err;
    }

    CurIOVal.u32=0x0;
    /* setup expender   i/o direction  all output but exp1 bit 14*/
    ExpanderData[0] = 0xFF;
    ExpanderData[1] = 0xFF;
    status = _ExpanderWR(I2cExpAddr0, GPDR, ExpanderData, 2);
    if (status) {
        XNUCLEO53L3A2_ErrLog("Set Expander @0x%02X DR", I2cExpAddr0);
        goto done_err;
    }
    ExpanderData[0] = 0xFF;
    ExpanderData[1] = 0xBF; // all but bit 14-15 that is pb1 and xhurt
    status = _ExpanderWR(I2cExpAddr1, GPDR, ExpanderData, 2);
    if (status) {
        XNUCLEO53L3A2_ErrLog("Set Expander @0x%02X DR", I2cExpAddr1);
        goto done_err;
    }
    /* shut down all segment and all device */
    CurIOVal.u32=0x7F + (0x7F<<7) + (0x7F<<16)+(0x7F<<(16+7));
    status= _ExpandersSetAllIO();
    if( status ){
        XNUCLEO53L3A2_ErrLog("Set initial i/o ");
    }

done_err:
    return status;
}


int XNUCLEO53L3A2_GetPB1(int *state) {
    int status;
    uint8_t  PortValue;
    status= _ExpanderRd(I2cExpAddr1, GPMR+1, &PortValue,1);
    if( status == 0){
        if( PortValue&=0x40 )
            PortValue=1;
        else
            PortValue=0;
    }
    else{
        XNUCLEO53L3A2_ErrLog("i/o error");
    }
    *state = PortValue;
    return status;
}

int XNUCLEO53L3A2_ResetId(int DevNo, int state) {
    int status;
    switch( DevNo ){
    case XNUCLEO53L3A2_DEV_CENTER :
    case 'c' :
        CurIOVal.bytes[3]&=~0x80; /* bit 15 expender 1  => byte #3 */
        if( state )
            CurIOVal.bytes[3]|=0x80; /* bit 15 expender 1  => byte #3 */
        status= _ExpanderWR(I2cExpAddr1, GPSR+1, &CurIOVal.bytes[3], 1);
        break;
    case XNUCLEO53L3A2_DEV_LEFT :
    case 'l' :
        CurIOVal.bytes[1]&=~0x40; /* bit 14 expender 0 => byte #1*/
        if( state )
            CurIOVal.bytes[1]|=0x40; /* bit 14 expender 0 => byte #1*/
        status= _ExpanderWR(I2cExpAddr0, GPSR+1, &CurIOVal.bytes[1], 1);
        break;
    case 'r' :
    case XNUCLEO53L3A2_DEV_RIGHT :
        CurIOVal.bytes[1]&=~0x80; /* bit 15 expender 0  => byte #1 */
        if( state )
            CurIOVal.bytes[1]|=0x80; /* bit 15 expender 0 => byte #1*/
        status= _ExpanderWR(I2cExpAddr0, GPSR+1, &CurIOVal.bytes[1], 1);
        break;
    default:
        XNUCLEO53L3A2_ErrLog("Invalid DevNo %d",DevNo);
        status = -1;
        goto done;
    }
//error with valid id
    if( status ){
        XNUCLEO53L3A2_ErrLog("expander i/o error for DevNo %d state %d ",DevNo, state);
    }
done:
    return status;
}


void VL53L3A2_EXTI_IOConfigure(int DevNo, int  IntPriority, int SubPriority){
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = VL53L3A2_INTR_PIN_PUPD;

    switch (DevNo ) {
    case XNUCLEO53L3A2_DEV_CENTER:
    case 'c':
        VL53L3A2_GPIO1_C_CLK_ENABLE();
        /*Configure GPIO pin : PA4 */
        GPIO_InitStruct.Pin = VL53L3A2_GPIO1_C_GPIO_PIN;

        XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_CENTER);
        HAL_GPIO_Init(VL53L3A2_GPIO1_C_GPIO_PORT, &GPIO_InitStruct);
        HAL_NVIC_SetPriority((IRQn_Type)VL53L3A2_GPIO1_C_GPIO_PIN, IntPriority, SubPriority);
        break;

#if VL53L3A2_GPIO1_SHARED == 0
    case XNUCLEO53L3A2_DEV_LEFT:
    case 'l':
        VL53L3A2_GPIO1_L_CLK_ENABLE();
        XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_LEFT);
        GPIO_InitStruct.Pin = VL53L3A2_GPIO1_L_GPIO_PIN;
        HAL_GPIO_Init(VL53L3A2_GPIO1_L_GPIO_PORT, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(VL53L3A2_GPIO1_L_INTx, IntPriority, SubPriority);
        break;

    case XNUCLEO53L3A2_DEV_RIGHT:
        VL53L3A2_GPIO1_R_CLK_ENABLE();
        XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_RIGHT);
        GPIO_InitStruct.Pin = VL53L3A2_GPIO1_R_GPIO_PIN;
        HAL_GPIO_Init(VL53L3A2_GPIO1_R_GPIO_PORT, &GPIO_InitStruct);

        /* note that L and R are shared group on l053 i/o as interrupt*/
        HAL_NVIC_SetPriority(VL53L3A2_GPIO1_R_INTx, IntPriority, SubPriority);
        break;
#endif
    }
}

void VL53L3A2_EXTI_IOUnconfigure(int DevNo){
    switch (DevNo ) {
    case XNUCLEO53L3A2_DEV_CENTER:
    case 'c':
        //XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_TOP);
        HAL_GPIO_DeInit(VL53L3A2_GPIO1_C_GPIO_PORT, VL53L3A2_GPIO1_C_GPIO_PIN);
        __HAL_GPIO_EXTI_CLEAR_IT(VL53L3A2_GPIO1_C_GPIO_PIN);
        break;

#if VL53L3A2_GPIO1_SHARED == 0
    case XNUCLEO53L3A2_DEV_LEFT:
    case 'l':
       // XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_LEFT);
        HAL_GPIO_DeInit(VL53L3A2_GPIO1_L_GPIO_PORT, VL53L3A2_GPIO1_L_GPIO_PIN);
        __HAL_GPIO_EXTI_CLEAR_IT(VL53L3A2_GPIO1_L_GPIO_PIN);
        break;

    case XNUCLEO53L3A2_DEV_RIGHT:
    case 'r':
        HAL_GPIO_DeInit(VL53L3A2_GPIO1_R_GPIO_PORT, VL53L3A2_GPIO1_R_GPIO_PIN);
        __HAL_GPIO_EXTI_CLEAR_IT(VL53L3A2_GPIO1_R_GPIO_PIN);
        //XNUCLEO53L3A2_SetIntrStateId(0,XNUCLEO53L3A2_DEV_RIGHT);
        break;
#endif
    }
}
/**
 * Set all i2c expended gpio in one go
 * @return i/o operation status
 */
static int _ExpandersSetAllIO(void){
    int status;
    status = _ExpanderWR(I2cExpAddr0, GPSR, &CurIOVal.bytes[0], 2);
    if( status ){
        goto done_err;
    }
    status = _ExpanderWR(I2cExpAddr1, GPSR, &CurIOVal.bytes[2], 2);
done_err:
    return status;
}

/**
 * STMPE1600  i2c Expender register read
 * @param I2cExpAddr Expender address
 * @param index      register index
 * @param data       read data buffer
 * @param n_data     number of byte to read
 * @return           of if ok else i2c I/O operation status
 */
static int _ExpanderRd(int I2cExpAddr, int index, uint8_t *data, int n_data) {

    int status;
    uint8_t RegAddr;
    RegAddr = index;
    XNUCLEO53L3A2_GetI2cBus();
    do {
        status = HAL_I2C_Master_Transmit(&XNUCLEO53L3A2_hi2c, I2cExpAddr, &RegAddr, 1, 100);
        if (status)
            break;
        status = HAL_I2C_Master_Receive(&XNUCLEO53L3A2_hi2c, I2cExpAddr, data, n_data, n_data * 100);
    } while (0);
    XNUCLEO53L3A2_PutI2cBus();
    return status;
}

/**
 * STMPE1600 i2c Expender register write
 * @param I2cExpAddr Expender address
 * @param index      register index
 * @param data       data buffer
 * @param n_data     number of byte to write
 * @return           of if ok else i2c I/O operation status
 */
static int _ExpanderWR(int I2cExpAddr, int index, uint8_t *data, int n_data) {

    int status;
    uint8_t RegAddr[0x10];
    RegAddr[0] = index;
    memcpy(RegAddr + 1, data, n_data);
    XNUCLEO53L3A2_GetI2cBus();
    status = HAL_I2C_Master_Transmit(&XNUCLEO53L3A2_hi2c, I2cExpAddr, RegAddr, n_data + 1, 100);
    XNUCLEO53L3A2_PutI2cBus();
    return status;
}


/**
 * @defgroup XNUCLEO53L3A2_7Segment 7 segment display
 *
 * macro use for human readable segment building
 * @code
 *  --s0--
 *  s    s
 *  5    1
 *  --s6--
 *  s    s
 *  4    2
 *  --s3-- . s7 (dp)
 * @endcode
 *
 * @{
 */
/** decimal point bit mapping*  */
#define DP  (1<<7)

//VL6180 shield
//#define S0 (1<<0)
//#define S1 (1<<1)
//#define S2 (1<<2)
//#define S3 (1<<3)
//#define S4 (1<<4)
//#define S5 (1<<5)
//#define S6 (1<<6)

/** sgement s0 bit mapping*/
#define S0 (1<<3)
/** sgement s1 bit mapping*/
#define S1 (1<<5)
/** sgement s2 bit mapping*/
#define S2 (1<<6)
/** sgement s3 bit mapping*/
#define S3 (1<<4)
/** sgement s4 bit mapping*/
#define S4 (1<<0)
/** sgement s5 bit mapping*/
#define S5 (1<<1)
/** sgement s6 bit mapping*/
#define S6 (1<<2)

/**
 * build a character by defining the non lighted segment (not one and no DP)
 *
 * @param  ... literal sum and or combine of any macro to define any segment #S0 .. #S6
 *
 * example '9' is all segment on but S4
 * @code
 *   ['9']=           NOT_7_NO_DP(S4),
 * @endcode
 */
#define NOT_7_NO_DP( ... ) (uint8_t) ~( __VA_ARGS__ + DP )

/**
 * Ascii to 7 segment  lookup table
 *
 * Most common character are supported and follow http://www.twyman.org.uk/Fonts/
 * few extra special \@ ^~ ... etc are present for specific demo purpose
 */
static const uint8_t ascii_to_display_lut[256]={
      [' ']=           0,
      ['-']=           S6,
      ['_']=           S3,
      ['=']=           S3+S6,
      ['~']=           S0+S3+S6, /* 3 h bar */
      ['^']=           S0, /* use as top bar */

      ['?']=           NOT_7_NO_DP(S5+S3+S2),
      ['*']=           NOT_7_NO_DP(),
      ['[']=           S0+S3+S4+S5,
      [']']=           S0+S3+S2+S1,
      ['@']=           S0+S3,

      ['0']=           NOT_7_NO_DP(S6),
      ['1']=           S1+S2,
      ['2']=           S0+S1+S6+S4+S3,
      ['3']=           NOT_7_NO_DP(S4+S5),
      ['4']=           S5+S1+S6+S2,
      ['5']=           NOT_7_NO_DP(S1+S4),
      ['6']=           NOT_7_NO_DP(S1),
      ['7']=           S0+S1+S2,
      ['8']=           NOT_7_NO_DP(0),
      ['9']=           NOT_7_NO_DP(S4),

      ['a']=           S2+ S3+ S4+ S6 ,
      ['b']=           NOT_7_NO_DP(S0+S1),
      ['c']=           S6+S4+S3,
      ['d']=           NOT_7_NO_DP(S0+S5),
      ['e']=           NOT_7_NO_DP(S2),
      ['f']=           S6+S5+S4+S0, /* same as F */
      ['g']=           NOT_7_NO_DP(S4), /* same as 9 */
      ['h']=           S6+S5+S4+S2,
      ['i']=           S4,
      ['j']=           S1+S2+S3+S4,
      ['k']=           S6+S5+S4+S2, /* a h */
      ['l']=           S3+S4,
      ['m']=           S0+S4+S2, /* same as  */
      ['n']=           S2+S4+S6,
      ['o']=           S6+S4+S3+S2,
      ['p']=           NOT_7_NO_DP(S3+S2), // same as P
      ['q']=           S0+S1+S2+S5+S6,
      ['r']=           S4+S6,
      ['s']=           NOT_7_NO_DP(S1+S4),
      ['t']=           NOT_7_NO_DP(S0+S1+S2),
      ['u']=           S4+S3+S2+S5+S1, // U
      ['v']=           S4+S3+S2, // is u but u use U
      ['w']=           S1+S3+S5,
      ['x']=           NOT_7_NO_DP(S0+S3), // similar to H
      ['y']=           NOT_7_NO_DP(S0+S4),
      ['z']=           S0+S1+S6+S4+S3, // same as 2

      ['A']=           NOT_7_NO_DP(S3),
      ['B']=           NOT_7_NO_DP(S0+S1), /* as b  */
      ['C']=           S0+S3+S4+S5, // same as [
      ['E']=           NOT_7_NO_DP(S1+S2),
      ['F']=           S6+S5+S4+S0,
      ['G']=           NOT_7_NO_DP(S4), /* same as 9 */
      ['H']=           NOT_7_NO_DP(S0+S3),
      ['I']=           S1+S2,
      ['J']=           S1+S2+S3+S4,
      ['K']=           NOT_7_NO_DP(S0+S3), /* same as H */
      ['L']=           S3+S4+S5,
      ['M']=           S0+S4+S2, /* same as  m*/
      ['N']=           S2+S4+S6, /* same as n*/
      ['O']=           NOT_7_NO_DP(S6),
      ['P']=           NOT_7_NO_DP(S3+S2),
      ['Q']=           NOT_7_NO_DP(S3+S2),
      ['R']=           S4+S6,
      ['S']=           NOT_7_NO_DP(S1+S4), /* sasme as 5 */
      ['T']=           NOT_7_NO_DP(S0+S1+S2), /* sasme as t */
      ['U']=           NOT_7_NO_DP(S6+S0),
      ['V']=           S4+S3+S2, // is u but u use U
      ['W']=           S1+S3+S5,
      ['X']=           NOT_7_NO_DP(S0+S3), // similar to H
      ['Y']=           NOT_7_NO_DP(S0+S4),
      ['Z']=           S0+S1+S6+S4+S3, // same as 2
};

#undef S0
#undef S1
#undef S2
#undef S3
#undef S4
#undef S5
#undef S6
#undef DP

/** @} */

int XNUCLEO53L3A2_SetDisplayString(const char *str) {
    int status;
    uint32_t Segments;
    int BitPos;
    int i;

    for( i=0; i<4 && str[i]!=0; i++){
        Segments = (uint32_t)ascii_to_display_lut[(uint8_t)str[i]];
        Segments =(~Segments)&0x7F;
        BitPos=DisplayBitPos[i];
        CurIOVal.u32 &=~(0x7F<<BitPos);
        CurIOVal.u32 |= Segments<<BitPos;
    }
    /* clear unused digit */
    for( ; i<4;i++){
        BitPos=DisplayBitPos[i];
        CurIOVal.u32 |=0x7F<<BitPos;
    }
    status = _ExpandersSetAllIO();
    if( status ){
        XNUCLEO53L3A2_ErrLog("Set i/o");
    }
    return status;
}

/**
 *
 * @}  XNUCLEO53L3A2_top
 */
