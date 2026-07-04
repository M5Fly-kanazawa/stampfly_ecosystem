
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */
unsigned int i2creadCount = 0;
unsigned int i2cwriteCount = 0;
unsigned char SPI2C_Buffer[256];



#include "vl53lx_platform.h"
#ifndef SMALL_FOOTPRINT
#include "vl53lx_platform_ipp.h"
#endif
#include <vl53lx_platform_log.h>

#include "vl53lx_api.h"

#include "stm32f4xx_hal.h"				 				 
#include <time.h>
#include <math.h>				 

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1



#ifdef VL53LX_LOG_ENABLE

#define trace_print(level, ...) \
	 _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	 level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	 _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	 VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

#endif

VL53LX_Error VL53LX_CommsInitialise(
	VL53LX_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error VL53LX_CommsClose(
	VL53LX_Dev_t *pdev)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}



#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif
//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
#ifndef VL53LX_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53LX_GetI2cBus(...) (void)0
#endif

#ifndef VL53LX_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53LX_PutI2cBus(...) (void)0
#endif



uint8_t _I2CBuffer[256];

int _I2CWrite(VL53LX_Dev_t *pdev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;
//    int i;
    i2cwriteCount+=count;
    status = HAL_I2C_Master_Transmit(pdev->I2cHandle, pdev->I2cDevAddr, pdata, count, i2c_time_out);

#ifdef VL53LX_LOG_ENABLE
    if (status) {
    	trace_print(VL53LX_TRACE_LEVEL_ERRORS, "HAL_I2C_Master_Transmit return %5u ;\n", status);
    }
#endif

    return status;
}

int _I2CRead(VL53LX_Dev_t *pdev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    i2creadCount+=count;
    status = HAL_I2C_Master_Receive(pdev->I2cHandle, pdev->I2cDevAddr|1, pdata, count, i2c_time_out);
#ifdef VL53LX_LOG_ENABLE
    if (status) {
    	trace_print(VL53LX_TRACE_LEVEL_ERRORS, "HAL_I2C_Master_Receive return %5u ;\n", status);
    }
#endif

    return status;
}



VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(pdev, _I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    VL53LX_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(pdev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdev, pdata, count);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53LX_PutI2cBus();
    return Status;
}




VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];


	buffer[0] = (uint8_t)(VL53LX_p_003);

	status = VL53LX_WriteMulti(pdev, index, buffer, 1);

	return status;
}


VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];


	buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
	buffer[1] = (uint8_t)(VL53LX_p_003 &  0x00FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	return status;
}


VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];


	buffer[0] = (uint8_t) (VL53LX_p_003 >> 24);
	buffer[1] = (uint8_t)((VL53LX_p_003 &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((VL53LX_p_003 &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (VL53LX_p_003 &  0x000000FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

	return status;
}


VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53LX_ReadMulti(pdev, index, buffer, 1);

	*pdata = buffer[0];

	return status;
}


VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);

	return status;
}


VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}



VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us)
{
	
	(void)pdev;
	HAL_Delay(wait_us/1000);
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{

	(void)pdev;
	HAL_Delay(wait_ms);
    return VL53LX_ERROR_NONE;
}



VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}




VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error  VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;

}


VL53LX_Error  VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}



VL53LX_Error  VL53LX_GpioXshutdown(uint8_t value)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error  VL53LX_GpioCommsSelect(uint8_t value)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error  VL53LX_GpioPowerEnable(uint8_t value)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error  VL53LX_GpioInterruptDisable(void)
{
	VL53LX_Error status = 255;
	/* To be filled by customer according to the platform request. Return 0 if OK */
	return status;
}


VL53LX_Error VL53LX_GetTickCount(
		VL53LX_Dev_t *pdev,
		uint32_t *ptick_count_ms)
{
	
    /* Returns current tick count in [ms] */
	
	VL53LX_Error status  = VL53LX_ERROR_NONE;
	(void) pdev;
	*ptick_count_ms = HAL_GetTick();
	//*ptick_count_ms = 0;

#ifdef VL53LX_LOG_ENABLE
	trace_print(
		VL53LX_TRACE_LEVEL_DEBUG,
		"VL53LX_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);
#endif

	return status;
	

}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{

	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint8_t      trace_functions = VL53LX_TRACE_FUNCTION_NONE;
#endif

	char   register_name[VL53LX_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
	VL53LX_get_register_name(
			index,
			register_name);
#else
	VL53LX_COPYSTRING(register_name, "");
#endif

	/* Output to I2C logger for FMT/DFT  */
#ifdef VL53LX_LOG_ENABLE
    trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, index, value, mask, poll_delay_ms);
	/*trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, register_name, value, mask, poll_delay_ms);*/
#endif
	/* calculate time limit in absolute time */

	 VL53LX_GetTickCount(pdev, &start_time_ms);

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53LX_LOG_ENABLE
	trace_functions = VL53LX_get_trace_functions();
	VL53LX_set_trace_functions(VL53LX_TRACE_FUNCTION_NONE);
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53LX_ERROR_NONE) &&
		   (polling_time_ms < timeout_ms) &&
		   (found == 0)) {

		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_RdByte(
							pdev,
							index,
							&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53LX_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53LX_WaitMs(
					pdev,
					poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53LX_GetTickCount(pdev, &current_time_ms);
		polling_time_ms = current_time_ms - start_time_ms;

	}

#ifdef VL53LX_LOG_ENABLE
	/* Restore function logging */
	VL53LX_set_trace_functions(trace_functions);
#endif

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;


}
