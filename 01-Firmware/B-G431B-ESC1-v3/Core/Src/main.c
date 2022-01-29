/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial.h"
#include "position_sensor.h"
#include "foc.h"
#include "pid.h"
#include "math_tool.h"
#include "eeprom.h"
#include "protocol.h"
#include "control_table.h"
#include "binary_tool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// CAN Bus termination
#define CAN_BUS_TERMINATION_OFF
#define CAN_BUS_TERMINATION_ON

// Position sensor type :
//    "AS5600_I2C"
//    "AS5048A_PWM"
#define SENSOR_TYPE AS5048A_PWM
//#define SENSOR_TYPE AS5600_I2C

// PID loop period in µs
//  normal setting is 1000us (1KHz)
//  performance setting is 250 (4KHz)
#define PID_LOOP_PERIOD 250

// FOC service loop period in µs
//  normal setting is 10000us (100Hz)
#define SERVICE_LOOP_PERIOD 10000

// Autocalibration at startup
//   uncomment this line for calibrating the ESC/MOTOR at startup
//   comment this line to avoid wearing EEPROM
//#define PERFORM_AUTO_CALIBRATION_AT_STARTUP

// Advanced settings (do not change)
#define ALPHA_VELOCITY				0.24f // (default:0.24) F = 1000Hz ==> Fc (-3dB) = 20Hz

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//extern float potentiometer_input_adc;

// serial communication
HAL_Serial_Handler serial;

// CAN communication, state and fail-safe
static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
static FDCAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];
bool can_armed = false;
uint32_t can_last_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PWM input capture IT for AS5048A position sensor
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) __attribute__((section (".ccmram")));

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (positionSensor_getType() == AS5048A_PWM)
	{
	  API_AS5048A_Position_Sensor_It(htim);
	}
}

// CAN configuration (filters)
static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x100;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(CAN_SHDN_GPIO_Port, CAN_SHDN_Pin, GPIO_PIN_RESET);
#ifdef CAN_BUS_TERMINATION_ON
  HAL_GPIO_WritePin(CAN_TERM_GPIO_Port, CAN_TERM_Pin, GPIO_PIN_SET); // SET means activating R120 for CAN bus termination
#else
  HAL_GPIO_WritePin(CAN_TERM_GPIO_Port, CAN_TERM_Pin, GPIO_PIN_RESET); // RESET means no bus termination
#endif

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

// CAN IT on message receive
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
  	// Handle CAN communication
  	while( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0)!=0)

    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
		// decode message ID=0x000+ID
		if(RxHeader.Identifier==regs[REG_ID]) // message from host controller
		{
		  // can watchdog re-arm
		  if( can_armed )
		  {
			  can_last_time = HAL_GetTick();
		  }
		  uint32_t payload_length = RxHeader.DataLength>>16U;
		  // check payload size = 8
		  if(can_armed && payload_length==2) // Feed Forward Torque only
		  {
			  // TODO Frist BYTE is CODE OP : 0:Torque, 1:VEL 2:POS  N:Is Write M:is Read FF is start of enable

			  // decode payload filed
			  regs[REG_GOAL_TORQUE_CURRENT_MA_L] = RxData[0];
			  regs[REG_GOAL_TORQUE_CURRENT_MA_H] = RxData[1];
			  //HAL_Serial_Print(&serial,"CAN (2)\n");
		  }
		  else if(can_armed && payload_length==3) // Speed, VEL Kp
		  {
			  // decode payload filed
			  regs[REG_GOAL_VELOCITY_DPS_L] = RxData[0];
			  regs[REG_GOAL_VELOCITY_DPS_H] = RxData[1];
			  regs[REG_GOAL_VEL_KP]  = RxData[4];
			  //HAL_Serial_Print(&serial,"CAN (3)\n");
		  }
		  else if(can_armed && payload_length==5) // Position, POS Kp and Kd, VEL Kp
		  {
			  // decode payload filed
			  regs[REG_GOAL_POSITION_DEG_L] = RxData[0];
			  regs[REG_GOAL_POSITION_DEG_H] = RxData[1];
			  regs[REG_GOAL_POS_KP]  = RxData[3];
			  regs[REG_GOAL_POS_KD]  = RxData[4];
			  regs[REG_GOAL_VEL_KP]  = RxData[5];
			  //HAL_Serial_Print(&serial,"CAN (5)\n");
		  }
		  else if(payload_length==8) // position, speed, and torque feed forward, Kp/kd update
		  {
			  if( (RxData[0]==0xFF) && (RxData[1]==0xFF) && (RxData[2]==0xFF) && (RxData[3]==0xFF) &&
				  (RxData[4]==0xFF) && (RxData[5]==0xFF) && (RxData[6]==0xFF) && (RxData[7]==0xFF) )
			  {
				  // init watch dog
				  can_armed = true;
				  can_last_time = HAL_GetTick();
				  regs[REG_TORQUE_ENABLE] = 1;
				  regs[REG_GOAL_POSITION_DEG_L] = 0;
				  regs[REG_GOAL_POSITION_DEG_H] = 0;
				  regs[REG_GOAL_VELOCITY_DPS_L] = 0;
				  regs[REG_GOAL_VELOCITY_DPS_H] = 0;
				  regs[REG_GOAL_TORQUE_CURRENT_MA_L] = 0;
				  regs[REG_GOAL_TORQUE_CURRENT_MA_H] = 0;
				  regs[REG_GOAL_POS_KP]  = 0;
				  regs[REG_GOAL_POS_KD]  = 0;
				  regs[REG_GOAL_VEL_KP]  = 0;
				  //HAL_Serial_Print(&serial,"CAN request ARM\n");
			  }
			  else if(can_armed)
			  {
				  // replace by access read/write to register !


				  // decode payload filed
				  regs[REG_GOAL_POSITION_DEG_L] = RxData[0];
				  regs[REG_GOAL_POSITION_DEG_H] = RxData[1];
				  regs[REG_GOAL_VELOCITY_DPS_L] = RxData[2];
				  regs[REG_GOAL_VELOCITY_DPS_H] = RxData[3];
				  regs[REG_GOAL_TORQUE_CURRENT_MA_L] = RxData[4];
				  regs[REG_GOAL_TORQUE_CURRENT_MA_H] = RxData[5];
				  regs[REG_GOAL_POS_KP]  = RxData[6];
				  regs[REG_GOAL_POS_KD]  = RxData[7];
				  //HAL_Serial_Print(&serial,"CAN (8)\n");
			  }
		  }

		  // then reply by a status frame (shortened)
		  TxHeader.Identifier = 0x10+regs[REG_ID]; // each ESC replies with a message identifier = it is own ID
		  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
		  TxData[0] = regs[REG_PRESENT_POSITION_DEG_L];
		  TxData[1] = regs[REG_PRESENT_POSITION_DEG_H];
		  TxData[2] = regs[REG_PRESENT_TORQUE_CURRENT_MA_L];
		  TxData[3] = regs[REG_PRESENT_TORQUE_CURRENT_MA_H];
		  //TxData[4] = regs[REG_HARDWARE_ERROR_STATUS];
		  //TxData[5] = regs[REG_PRESENT_VOLTAGE];
		  //TxData[6] = regs[REG_PRESENT_TEMPERATURE];
		  //TxData[7] = ....
		  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,TxData);
		}
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6); // 1us base time
	API_FOC_Init();

  if(eeprom_empty())
	factory_reset_eeprom_regs();
  load_eeprom_regs();
  reset_ram_regs();
  FDCAN_Config();
  positionSensor_init(SENSOR_TYPE);
  positionSensor_update();
  API_FOC_Service_Update();
#ifdef PERFORM_AUTO_CALIBRATION_AT_STARTUP
  API_FOC_Calibrate();
#endif
  HAL_Serial_Init(&huart2,&serial);
//HAL_Serial_Print(&serial,"RESET!\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool torque_was_enable = false;
	uint16_t present_time_us = __HAL_TIM_GET_COUNTER(&htim6);
	uint16_t pid_last_time_us = present_time_us;
	uint16_t service_last_time_us = present_time_us;
	float setpoint_position_deg = 0.0f;
	float setpoint_velocity_dps = 0.0f;
	float setpoint_torque_current_mA = 0.0f;
	float setpoint_flux_current_mA = 0.0f;
	float error_velocity_dps = 0.0f;
	uint32_t pid_counter = 0;
	uint32_t mlp_counter = 0;
	pid_context_t pd_position;
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	// hardware error clears torque enable register and disable FOC torque controller
		if( regs[REG_HARDWARE_ERROR_STATUS] != 0 )
		{
			// disable torque
			regs[REG_TORQUE_ENABLE] = 0;
			// disable FOC
			API_FOC_Torque_Disable();
			// disarm CAN
			can_armed = false;
		}

		// CAN bus time-out
		// CAN bus watchdog (time-out = 1s hard-coded)
		if( can_armed && (HAL_GetTick()>can_last_time+1000) )
		{
			// disable torque
			regs[REG_TORQUE_ENABLE] = 0;
			// disable FOC
			API_FOC_Torque_Disable();
			// disarm CAN
			can_armed = false;
		}

		// FOC process time-out
		// FOC watchdog (time-out = 10ms hard-coded)
		if( HAL_GetTick() > API_FOC_Get_Timestamp_ms()+10 )
		{
			// error
			regs[REG_HARDWARE_ERROR_STATUS] |= (1 << HW_ERROR_BIT_FOC_TIMEOUT);
			// disable torque
			regs[REG_TORQUE_ENABLE] = 0;
			// disable FOC
			API_FOC_Torque_Disable();
			// disarm CAN
			can_armed = false;
		}

		// 1 to 4Khz low priority process
		present_time_us = __HAL_TIM_GET_COUNTER(&htim6);
		uint16_t const pid_delta_time_us = present_time_us-pid_last_time_us;
		if(pid_delta_time_us>=PID_LOOP_PERIOD)
		{
			pid_last_time_us+=PID_LOOP_PERIOD;
			++pid_counter;

			// update sensor
			positionSensor_update();

			if(regs[REG_TORQUE_ENABLE])
			{
				// transition from torque disable to enable
				if(!torque_was_enable)
				{
					torque_was_enable = true;
					// set goal position to present position to avoid mechanical glicth
					regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(10.0f*positionSensor_getDegreeMultiturn()));
					regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(10.0f*positionSensor_getDegreeMultiturn()));
					// reset goal velocity
					regs[REG_GOAL_VELOCITY_DPS_L] = 0;
					regs[REG_GOAL_VELOCITY_DPS_H] = 0;
					// reset feed-forward torque
					regs[REG_GOAL_TORQUE_CURRENT_MA_L] = 0;
					regs[REG_GOAL_TORQUE_CURRENT_MA_H] = 0;
					// reset flux refenrece
					regs[REG_GOAL_FLUX_CURRENT_MA_L] = 0;
					regs[REG_GOAL_FLUX_CURRENT_MA_H] = 0;
					// reset K
					regs[REG_GOAL_POS_KP] = 0;
					regs[REG_GOAL_POS_KD] = 0;
					regs[REG_GOAL_VEL_KP] = 0;
					// reset setpoints
					setpoint_position_deg = 0.0f;
					setpoint_velocity_dps = 0.0f;
					error_velocity_dps = 0.0f;
					// set setpoint_position_deg to avoid glitch
					setpoint_position_deg = positionSensor_getDegreeMultiturn();
					// pid reset
					pid_reset(&pd_position);
					// foc
					API_FOC_Torque_Enable();
				}



				// compute position set-point from goal and EEPROM position limits
				float const goal_position_deg = (float)((int16_t)(MAKE_SHORT(regs[REG_GOAL_POSITION_DEG_L],regs[REG_GOAL_POSITION_DEG_H])))/10.0f;
				float const reg_min_position_deg = (float)((int16_t)(MAKE_SHORT(regs[REG_MIN_POSITION_DEG_L],regs[REG_MIN_POSITION_DEG_H])));
				float const reg_max_position_deg = (float)((int16_t)(MAKE_SHORT(regs[REG_MAX_POSITION_DEG_L],regs[REG_MAX_POSITION_DEG_H])));
				setpoint_position_deg = fconstrain(goal_position_deg,reg_min_position_deg,reg_max_position_deg);
				// compute velocity setpoint from goal and EEPROM velocity limit
				float const goal_velocity_dps = (int16_t)(MAKE_SHORT(regs[REG_GOAL_VELOCITY_DPS_L],regs[REG_GOAL_VELOCITY_DPS_H]));
				float const reg_max_velocity_dps = (int16_t)(MAKE_SHORT(regs[REG_MAX_VELOCITY_DPS_L],regs[REG_MAX_VELOCITY_DPS_H]));
				setpoint_velocity_dps = fconstrain(goal_velocity_dps,-reg_max_velocity_dps,reg_max_velocity_dps);
				// compute torque feed forward
				float const torque_feed_forward_ma = (int16_t)(MAKE_SHORT(regs[REG_GOAL_TORQUE_CURRENT_MA_L],regs[REG_GOAL_TORQUE_CURRENT_MA_H]));
				// compute torque setpoint
				float const error_position_deg = setpoint_position_deg-positionSensor_getDegreeMultiturn();
				float const pos_kp = (float)regs[REG_GOAL_POS_KP]*2.0f;
				float const pos_kd = (float)regs[REG_GOAL_POS_KD]*100.0f*2.0f;
				float const reg_max_current_ma = (uint16_t)(MAKE_SHORT(regs[REG_MAX_CURRENT_MA_L],regs[REG_MAX_CURRENT_MA_H]));
				float const vel_kp = (float)regs[REG_GOAL_VEL_KP]/10.0f;
				error_velocity_dps = ALPHA_VELOCITY*(setpoint_velocity_dps-positionSensor_getVelocityDegree())+(1.0f-ALPHA_VELOCITY)*error_velocity_dps;
				float const reg_reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
				setpoint_torque_current_mA = reg_reverse*pid_process_antiwindup_clamp_with_ff(
						&pd_position,
						error_position_deg,
						pos_kp,
						0.0f,
						pos_kd,
						reg_max_current_ma,
						0.1f,
						vel_kp*error_velocity_dps+torque_feed_forward_ma
				);
				//setpoint_torque_current_mA=(potentiometer_input_adc/4096)*3000.0f; // DEBUG
				// set flux
				float const goal_flux_current_mA = (int16_t)(MAKE_SHORT(regs[REG_GOAL_FLUX_CURRENT_MA_L],regs[REG_GOAL_FLUX_CURRENT_MA_H]));
				setpoint_flux_current_mA = goal_flux_current_mA;

				// update FOC parameters
				API_FOC_Set_Torque_Flux_Currents_mA(setpoint_torque_current_mA,setpoint_flux_current_mA);

			}
			else // torque disable
			{
				// transition from enable to disable
				if(torque_was_enable)
				{
					torque_was_enable = false;
					// disable FOC
					API_FOC_Torque_Disable();
				}

				// reset unused RAM
				regs[REG_GOAL_POSITION_DEG_L] = 0;
				regs[REG_GOAL_POSITION_DEG_H] = 0;
				regs[REG_GOAL_VELOCITY_DPS_L] = 0;
				regs[REG_GOAL_VELOCITY_DPS_H] = 0;
				regs[REG_GOAL_TORQUE_CURRENT_MA_L] = 0;
				regs[REG_GOAL_TORQUE_CURRENT_MA_H] = 0;
				regs[REG_GOAL_FLUX_CURRENT_MA_L] = 0;
				regs[REG_GOAL_FLUX_CURRENT_MA_H] = 0;
				regs[REG_GOAL_POS_KP] = 0;
				regs[REG_GOAL_POS_KD] = 0;
				regs[REG_GOAL_VEL_KP] = 0;

				// reset all setpoints
				setpoint_position_deg = 0.0f;
				setpoint_velocity_dps = 0.0f;
				error_velocity_dps = 0.0f;

				setpoint_torque_current_mA = 0.0f;
				setpoint_flux_current_mA = 0.0f;
			}

			// RAM Update
			regs[REG_PRESENT_POSITION_DEG_L] = LOW_BYTE((int16_t)(positionSensor_getDegreeMultiturn()*10.0f));
			regs[REG_PRESENT_POSITION_DEG_H] = HIGH_BYTE((int16_t)(positionSensor_getDegreeMultiturn()*10.0f));
			regs[REG_PRESENT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)(positionSensor_getVelocityDegree()*1.0f));
			regs[REG_PRESENT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)(positionSensor_getVelocityDegree()*1.0f));
			regs[REG_PRESENT_TORQUE_CURRENT_MA_L] = LOW_BYTE((int16_t)(API_FOC_Get_Present_Torque_Current()*1.0f));
			regs[REG_PRESENT_TORQUE_CURRENT_MA_H] = HIGH_BYTE((int16_t)(API_FOC_Get_Present_Torque_Current()*1.0f));
			regs[REG_PRESENT_FLUX_CURRENT_MA_L] = LOW_BYTE((int16_t)(API_FOC_Get_Present_Flux_Current()*1.0f));
			regs[REG_PRESENT_FLUX_CURRENT_MA_H] = HIGH_BYTE((int16_t)(API_FOC_Get_Present_Flux_Current()*1.0f));
			regs[REG_PRESENT_VOLTAGE] = (uint16_t)(API_FOC_Get_Present_Voltage());
			regs[REG_PRESENT_TEMPERATURE] = (uint16_t)(API_FOC_Get_Present_Temp());
			regs[REG_MOVING] = (uint16_t)(fabsf(positionSensor_getVelocityDegree())) > (uint16_t)(regs[REG_MOVING_THRESHOLD_DPS]) ? 1 : 0;

			// DEBUG RAM Update
			regs[REG_SETPOINT_POSITION_DEG_L] = LOW_BYTE((int16_t)(setpoint_position_deg*10.0f));
			regs[REG_SETPOINT_POSITION_DEG_H] = HIGH_BYTE((int16_t)(setpoint_position_deg*10.0f));
			regs[REG_SETPOINT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)(setpoint_velocity_dps*1.0f));
			regs[REG_SETPOINT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)(setpoint_velocity_dps*1.0f));
			regs[REG_SETPOINT_TORQUE_CURRENT_MA_L] = LOW_BYTE((int16_t)(setpoint_torque_current_mA*1.0f));
			regs[REG_SETPOINT_TORQUE_CURRENT_MA_H] = HIGH_BYTE((int16_t)(setpoint_torque_current_mA*1.0f));
			regs[REG_SETPOINT_FLUX_CURRENT_MA_L] = LOW_BYTE((int16_t)(setpoint_flux_current_mA*1.0f));
			regs[REG_SETPOINT_FLUX_CURRENT_MA_H] = HIGH_BYTE((int16_t)(setpoint_flux_current_mA*1.0f));
			// test new FW
			regs[REG_PROCESSING_TIME] = (uint8_t)(API_FOC_Get_Processing_Time());
			regs[REG_FOC_FREQUENCY] = (uint8_t)(API_FOC_Get_Processing_Frequency()/1000.0f);
			regs[REG_PID_FREQUENCY] = (uint8_t)((float)pid_counter/(float)HAL_GetTick());
			regs[REG_MLP_FREQUENCY] = (uint8_t)((float)mlp_counter/(float)HAL_GetTick());

		} // low priority process 4Khz

		// low priority low frequency
		uint16_t const service_delta_time_us = present_time_us-service_last_time_us;
		if(service_delta_time_us>=SERVICE_LOOP_PERIOD)
		{
			service_last_time_us+=SERVICE_LOOP_PERIOD;

			// FOC service update
			API_FOC_Service_Update();

			// Handle local MMI
			// Led STATUS
			HAL_GPIO_WritePin(STATUS_GPIO_Port,STATUS_Pin,(regs[REG_LED]>0)||(regs[REG_HARDWARE_ERROR_STATUS]>0)?GPIO_PIN_SET:GPIO_PIN_RESET);

			// DEBUG
//			if(potentiometer_input_adc>200)
//				regs[REG_TORQUE_ENABLE] = 1;
//			else
//				regs[REG_TORQUE_ENABLE] = 0;

			// Pressing the button starts calibration
			if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin)==GPIO_PIN_RESET)
			{
				// disable torque
				regs[REG_TORQUE_ENABLE] = 0;
				// disable FOC
				API_FOC_Torque_Disable();
				// disarm CAN
				can_armed = false;
				// start FOC calibration
				API_FOC_Calibrate();
			}

			// Handle serial communication
			while(HAL_Serial_Available(&serial))
			{
			  char c = HAL_Serial_GetChar(&serial);
			  packet_handler(c);
			}
		} // low priority process very low frequency

		// PERFORMANCE
		//uint16_t t_end = __HAL_TIM_GET_COUNTER(&htim6);
		++mlp_counter;

		// TRACE
		static uint32_t counter = 0;
		if(((++counter)%100)==0)
		{
//			HAL_Serial_Print(&serial,"%d %d\n",
//						//(int)(RADIANS_TO_DEGREES(positionSensor_getRadians())*10.0f),
//						(int)(init_error_data_bits),
//						(int)(RADIANS_TO_DEGREES(present_position_rad)*10.0f)
//						//(int)(RADIANS_TO_DEGREES(expected_position)*10.0f)
//						//(int)(RADIANS_TO_DEGREES(present_velocity_rad)*0.1f)
//						//(int)(RADIANS_TO_DEGREES(theta_rad)*10.0f)
//						//(int)(RADIANS_TO_DEGREES(absolute_position_rad)*10.0f)
//						//regs[REG_PROTOCOL_CRC_FAIL]
//						//(int)(RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_RPS())*10.0f)
//						//(int)positionSensor_getDeltaTimeEstimation()
//					);


//			HAL_Serial_Print(&serial,"%d %d %d\n",
//						(int)(setpoint_torque_current_mA),
//						(int)(API_FOC_Get_Present_Torque_Current()),
//						(int)(API_FOC_Get_Present_Flux_Current())
//					);

//			HAL_Serial_Print(&serial,"%d\n",
//			(int)API_AS5048A_Position_Sensor_Get_DeltaTimestamp()
//			);
		}


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00701F6B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 128;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 159;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN_TERM_Pin|STATUS_Pin|CAN_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_TERM_Pin STATUS_Pin CAN_SHDN_Pin */
  GPIO_InitStruct.Pin = CAN_TERM_Pin|STATUS_Pin|CAN_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
