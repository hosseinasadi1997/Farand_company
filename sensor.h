/****************************************************************************
* Title                 :   Moor III Head Sensor Application 
* Filename              :   sensor.h
* Author                :   Homa
* Origin Date           :   1399/02/28
* Version               :   1.2
* Compiler              :   Microchip C30 v3.30c ???
* Target                :   STM32F746ZGT7
* Notes                 :   None
*****************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*     Date     Software Version     Initials     Description 
*  1399/02/20         1.0             Homa       Module Created.
*
*  1399/02/24         1.1             Homa       HDC sensor separated.
*
*  1399/02/28         1.2             Homa       Sensor structure defenition editted.
*
*****************************************************************************/
/** \file tsk.h
 *  \brief This module contains continuous tasks.
 * 
 *  This is the header file for the definition for tasks that are continuous
 *  and should be ran as often as possible.
 */
#ifndef SENSOR_H_
#define SENSOR_H_

#define DANGER 1
#define OKK 0
#define CHECKED 2
/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/**
 * This constant is
 */


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


	
/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum
{
	HEAD_PAN_MOTOR_CURRENT_SENSOR = 0,
	HEAD_TILT_MOTOR_CURRENT_SENSOR = 1,
	HEAD_INTERNAL_TEMP_HDC_SENSOR = 2,
	HEAD_INTERNAL_HMDTY_HDC_SENSOR = 3,
	HEAD_EXTERNAL_TEMP_HDC_SENSOR = 4,
	HEAD_EXTERNAL_HMDTY_HDC_SENSOR = 5,
	HEAD_TOP_TOF_SENSOR = 6,
	HEAD_FRONT_TOF_SENSOR = 7,
	HEAD_LEFT_TOF_SENSOR = 8,
	HEAD_RIGHT_TOF_SENSOR = 9,
	HEAD_CHIN_TOF_SENSOR = 10,
	HEAD_GAS_SENSOR = 11,
	HEAD_SMOKE_SENSOR = 12,
	HEAD_PRESSURE_SENSOR = 13
} SensorName_Enum_TypeDef;

typedef struct
{
	SensorName_Enum_TypeDef Name;
	uint32_t Value_Raw;
	float Gain;
	float Offset;
	float Value_Physical;
	float Value_Filtered;
	float Filter_Coeff;
	float Out_of_Range_Upper_Limit;
	float Out_of_Range_Lower_Limit;
	float Alarm_Upper_Limit;
	float Alarm_Lower_Limit;
	float Fault_Upper_Limit;
	float Fault_Lower_Limit;
	uint8_t Status;
}Sensor_ConfigStruct_TypeDef;


/******************************************************************************
* Variables
*******************************************************************************/
extern Sensor_ConfigStruct_TypeDef*		Internal_Temp_HDC_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Internal_Hmdty_HDC_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		External_Temp_HDC_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		External_Hmdty_HDC_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Top_TOF_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Front_TOF_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Left_TOF_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Right_TOF_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Chin_TOF_Sensor;
extern Sensor_ConfigStruct_TypeDef*		Gas_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Smoke_Sensor_Ptr;
extern Sensor_ConfigStruct_TypeDef*		Pressure_Sensor_Ptr;

extern int i2c1_state;
extern int i2c2_state;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

extern void Initialize_Current_Sensors(void);
extern void Check_Encoder(void);



extern void Initialize_Sensor_Struct(void);
extern void GetSensorsDataTask(void const * argument);
extern osThreadId getSensorsDataTaskHandle;
extern void ManageI2CBus(void);	
	
#ifdef __cplusplus
} // extern "C"
#endif

#endif /*SENSOR_H_*/

/*** End of File **************************************************************/
