/****************************************************************************
* Title                 :   Moor III Head Motor Application   
* Filename              :   motor_controller.h
* Author                :   Homa
* Origin Date           :   05/09/2020
* Version               :   3.0
* Compiler              :   Microchip C30 v3.30c   ???
* Target                :   STM32F746ZGT7
* Notes                 :   None
*****************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*     Date     Software Version     Initials     Description 
*  05/09/2020        3.0              Homa       Motor Structure updated.
*
*****************************************************************************/
/** \file motor_controller.h
 *  \brief This module contains all motor related part.
 * 
 *  This is the header file for the definition of motor structure and enumeration
 *  and contains variables' and functions' declaration and also indicates extern ones
 */
#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H


/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f7xx_hal.h"
#include "stdbool.h"
#include "main.h"
//#include "sensor.h"

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

#define CCW -1 
#define STOP 0
#define CW 1


#define SELF_TST_CW_MVT_STATE            0
#define SELF_TST_CW_CHK_STATE            1
#define SELF_TST_CCW_MVT_STATE           2
#define SELF_TST_CCW_CHK_STATE           3
#define CALIB_MVT_STATE                  4
#define FAST_GOTO_LIMIT_SWITCH_STATE     5
#define RETREAT_FROM_LIMIT_SWITCH_STATE  6
#define SLOW_GOTO_LIMIT_SWITCH_STATE     7
#define GOTO_FRONT_POSITION_STATE        8
#define NORMAL_OPERATION_STATE           9
#define SELF_TST_ERROR_STATE             10
#define SELF_TST_CW_WAIT_STATE           11
#define SELF_TST_CCW_WAIT_STATE          12
#define SELF_TEST_INIT_WAIT          		 13
#define MOTOR_JAMMED_STATE                     14
#define REPOSITION_TO_RIGHT        			 15
#define REPOSITION_TO_LEFT 	       			 16


#define PAN_LOWER_LIMIT_ANGLE   -119.5
#define PAN_Upper_LIMIT_ANGLE   147

typedef enum
{
	HEAD_PAN_MOTOR = 0,
	HEAD_TILT_MOTOR = 1
} MotorName_Enum_TypeDef;

typedef struct
{
	MotorName_Enum_TypeDef Name;
	uint8_t Encoder_Pulse_A;
 	uint8_t Encoder_Pulse_B;
	float Angle_Offset;
	float Angle_Gain;
	int32_t Measured_Angle_Count_Raw;
	int32_t Measured_Angle_Count_Raw_Prev;
	float Measured_Angle_Deg_Raw;
	float Measured_Angle_Count;
	float Measured_Angle_Count_Prev;
	float Measured_Angle_Deg;
	int32_t Desired_Angle_Count;
	float Desired_Angle_Deg;
	float Measured_Periodic_Angle_Deg;
	int32_t Error_Angle_Count;
	float Measured_Speed_Deg_Per_Sec_Raw;
	float Measured_Speed_Deg_Per_Sec_Raw_Prev;
	float Measured_Speed_Deg_Per_Sec;
	float Desired_Speed_Deg_Per_Sec;
	float Desired_Speed_Deg_Per_Sec_Prev;
	float Requested_Speed_Deg_Per_Sec_Prev;
	float Speed_Error_Deg_Per_Sec;
	float Angle_Error_Deg;
	int8_t Direction;	
	float Excitation_Value;
	uint8_t Speed_Code;
	uint8_t Time_Slot_Counter;
	uint8_t Time_Slot_Finished;
	int8_t Excitation_PWM;
	bool Driver_Fault;
 	bool Lower_Limit_Switch;
	bool Upper_Limit_Switch;
 	bool Lower_Limit_Switch_Prev;
	bool Upper_Limit_Switch_Prev;
	uint8_t Position_Control_Active_Flag;
	int8_t Position_System_State;
	int8_t Start_Update_State;
	uint16_t TimeOut_Counter;
	int8_t Motion_Direction;
	uint8_t Encoder_Fault;
	uint8_t Encoder_Check_Flag;
	uint8_t Check_Encoder_Direction;
	uint8_t Over_Current_Timer;
	uint8_t Over_Current_Error_Flag;
	Sensor_ConfigStruct_TypeDef Current;
	uint8_t Calibration_Complete_Flag;
	int8_t Position_System_State_Prev;
	float Speed_Ctrl_Time;
	int16_t Delta_PWM;
	
//	Sensor_ConfigStruct_TypeDef* Current;
	uint8_t Status;
}Motor_ConfigStruct_TypeDef;


/******************************************************************************
* Variables
*******************************************************************************/

extern int start_speedController_loop_flag;
extern int start_positionController_loop_flag;
extern int start_openLoop_control_flag;
extern int go_to_position_flag;
extern int start_calibration_flag;
extern int angle_error_count;
extern int z_start_flag;
extern int z_stop_flag;
extern float pi;
extern float lowPass_filter_coeff;
extern float speed_error_degPersec;
extern float delta_excitation;
extern int8_t speed_Controller_State;
extern uint16_t excitation_Max;
extern Motor_ConfigStruct_TypeDef	 Pan_Motor;
extern Motor_ConfigStruct_TypeDef	 Tilt_Motor;
extern uint8_t time_Slots_Finished;
extern float system_Time;
extern float Ts;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif
	
extern void Initialize_Motor_Struct(void);
extern void Apply_Direction_Commands(void);
extern void Read_Angle_Encoders(Motor_ConfigStruct_TypeDef *Motor);
extern void Report_Measured_Data(void);
extern void Apply_Pulsive_Control(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Apply_Position_Control(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Update_Position_And_Speed_Measurements(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Apply_Pulsive_Motion_Control(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Update_Position_System_State(void);
extern void Update_Head_Position_State(Motor_ConfigStruct_TypeDef *Motor);
extern void Evaluate_Motor_Current(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Check_Limit_Switches(void);
extern void Update_Desired_Angle(Motor_ConfigStruct_TypeDef *Motor_Ptr);
extern void Apply_Pulsive_Speed_Control(Motor_ConfigStruct_TypeDef *Motor_Ptr);


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */
/*** End of File **************************************************************/
