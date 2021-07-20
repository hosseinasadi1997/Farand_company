/****************************************************************************
* Title                 :   Moor III Head State Machine Application   
* Filename              :   state_machine.h
* Author                :   Homa
* Origin Date           :   05/13/2020
* Version               :   1.2
* Compiler              :   Microchip C30 v3.30c   ???
* Target                :   STM32F746ZGT7
* Notes                 :   None
*****************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*     Date     Software Version     Initials     Description 
*  02/20/2020        1.0             Homa        Module created.
*
*  05/09/2020        1.1             Homa        Template format applied.
*
*  05/13/2020        1.2             Homa        Some sub-states added.
*
*****************************************************************************/
/** \file state_machine.h
 *  \brief This module contains all state machine related part.
 * 
 *  This is the header file for the definition of system state enumeration
 *  and contains variables' and functions' declaration and also indicates extern ones
 */
#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H


/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f7xx_hal.h"
#include "motor_controller.h"

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
	STARTUP_STATE = 0,
	SELF_TEST_STATE = 1,
	CALIBRATION_STATE = 2,
	OPERATION_STATE = 3
} StateMachineName_Enum_TypeDef;

typedef enum 
{
	PAN_CCW_SELFTEST_SUBSTATE = 0,
	PAN_CW_SELFTEST_SUBSTATE = 1,
	WAIT_SELFTEST_SUBSTATE = 2,
	TILT_CCW_SELFTEST_SUBSTATE = 3,
	TILT_CW_SELFTEST_SUBSTATE = 4
} SelfTestSubStates_Enum_TypeDef;
/******************************************************************************
* Variables
*******************************************************************************/
extern uint8_t self_test_substate;
extern __IO uint16_t system_state_counter;
extern __IO uint8_t system_state;
extern __IO uint32_t startup_state_finished_flag;
extern uint8_t apply_self_test_pwm_flag;
extern uint8_t apply_calibration_pwm_flag;
extern uint8_t go_to_zero_position_flag;
extern int8_t start_increasing_pwm_flag;
extern float encoder_limit_base;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif
	
extern void RGB_Test(char color);
//void Update_System_State(void);
extern void Self_Test_Apply_PWM(Motor_ConfigStruct_TypeDef *Motor,int8_t self_test_pwm);
extern int Self_Test_Check_Encoder(Motor_ConfigStruct_TypeDef *Motor,int8_t self_test_motor_dir);
extern int Calibrate_Motor_Position(Motor_ConfigStruct_TypeDef *Motor);
extern void Go_To_Position(Motor_ConfigStruct_TypeDef *Motor,int angle_degree);
 

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H */
/*** End of File **************************************************************/
