/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pid.h"
#include "BNO055_STM32.h"
#include "rpi_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 로봇2용 펌웨어. 로봇1(robot1/main.c)과 구조는 같고, 모터 특성이 달라 튜닝값만 다르다 */
/* ---------------------------------------------------------------
 * Raspberry Pi -> STM32 명령 코드
 * 라즈베리파이가 보내는 <cmd,±xxxx,±yyyy,±yyyy>에서 맨 앞 cmd 자리에 들어간다.
 * 프레임 길이가 21바이트로 고정이라 cmd는 한 자리 숫자(0~9)만 쓸 수 있다.
 * --------------------------------------------------------------- */
#define CMD_FORWARD       0x01  // 다음 마커까지 직진
#define CMD_STOP          0x02  // 모터 정지
#define CMD_LIFT_UP       0x03  // 리프트 상승
#define CMD_LIFT_DOWN     0x04  // 리프트 하강
#define CMD_ROTATE_LEFT   0x05  // 제자리 좌회전 90도
#define CMD_ROTATE_RIGHT  0x06  // 제자리 우회전 90도
#define CMD_ROTATE_180    0x07  // 제자리 180도 회전
#define CMD_READY         0x08  // 미사용 (예약)

// 절댓값 매크로
// 주의: (-x)에 괄호가 없어서 ABS(a - b)처럼 식을 넣으면 -a - b로 바뀌어 계산이 틀린다.
//       지금은 변수 하나씩만 넣어서 쓰고 있어 문제없음.
#define ABS(x) (((x) > 0) ? (x) : (-x))

// BNO055 캘리브레이션 값(22바이트)을 저장해둔 플래시 주소 (Sector 11)
// 캘리브레이션용 펌웨어를 따로 돌려서 미리 저장해둔다.
#define CALIB_ADDR  ((uint32_t)0x081C0000)

/* 엔코더·바퀴 파라미터 (엔코더 타이머는 4체배 모드) */
#define MOTOR_CPR           64.0f   // 모터축 1회전당 카운트 (16 PPR x4)
#define GEAR_RATIO          70.0f   // 감속비 70:1
#define ENCODER_CPR         (MOTOR_CPR * GEAR_RATIO)   // 바퀴 1회전당 카운트
#define WHEEL_DIAMETER_MM   100.0f
#define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_DIAMETER_MM)  // mm
#define TARGET_DISTANCE     507.0f  // 한 구간 직진 거리 (mm). 선반을 들면 무거워서 덜 가기 때문에 마커 간격보다 늘려서 맞춘 값

/* 마커 값이 정상 범위인지 검사하는 기준
 * 통신 중 숫자가 깨져서 들어올 경우를 대비한 안전장치 (프레임에 오류 검사 값이 없어서) */
#define MARKER_X_MM_LIMIT       30.0f   // 마커 중심에서 x로 이만큼 넘게 벗어나면 카메라에 안 보이는 위치라 잘못된 값
#define MARKER_Y_MM_LIMIT       30.0f   // y도 같은 기준 (mm)
#define MARKER_DYAW_LIMIT_DEG   10.0f   // 각도 차이가 이 이상이면 잘못 읽은 값으로 보고 회전 보정에 안 씀 (deg)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* ---- AGV 상태 ---- */
volatile AGV_State_t state = STATE_READY;   // READY: 명령 대기, MOVE: 동작 중
volatile AGV_Command_t command = 0x00;      // 현재 실행 중인 명령
volatile AGV_Event_t event = 0x00;          // 미사용

/* ---- 인터럽트 -> 메인 루프 전달 플래그 ---- */
volatile uint8_t control_flag = 0,     // 10 ms마다 TIM6 인터럽트가 1로 만듦 → 메인 루프가 보고 제어 실행
                 print_flag = 0,       // 디버그 출력용 카운터 (10번 = 100 ms마다 출력)
                 usart2_flag = 0,      // 라즈베리파이 데이터를 정상으로 받으면 1
                 lift_done_flag = 0;   // 리프트 이동이 끝나면 1 (TIM5 인터럽트)
volatile uint8_t forward_align_done = 0;   // 직진 전에 방향 맞추기를 끝냈으면 1
int16_t MotorL_Speed = 0, MotorR_Speed = 0;   // 모터에 실제로 넣는 PWM 값
float Target_Yaw = 0.0f,         // 목표 yaw (deg)
      Target_Distance = 0.0f;    // 직진 목표 거리 (mm)

/* ---- 엔코더 (속도·거리 제어) ---- */
uint16_t EncoderL_PrevCount, EncoderR_PrevCount;    // 이전 주기 카운터 값
uint16_t EncoderL_CurrCount, EncoderR_CurrCount;    // 현재 주기 카운터 값
int16_t EncoderL_DeltaCount, EncoderR_DeltaCount;   // 10 ms 동안 늘어난 카운트 (= 바퀴 속도)

uint32_t EncoderL_TotalCount, EncoderR_TotalCount;  // 직진 시작 후 누적 카운트
uint32_t Encoder_TotalCount;                        // 좌우 평균 누적 카운트
float Total_Distance;                               // 누적 이동 거리 (mm)

PID_t MOTOR_L, MOTOR_R;   // 좌우 바퀴 속도 제어
PID_t DISTANCE;           // 이동 거리 제어

/* ---- BNO055 (방향 제어) ---- */
// 지자기 센서 영향을 피하기 위해 IMU 모드(가속도 + 자이로)로 동작
BNO055_Init_t bno055_init = {
		UNIT_ACC_MS2 | UNIT_GYRO_DPS | UNIT_EUL_DEG | UNIT_TEMP_CELCIUS,
		DEFAULT_AXIS_REMAP,
		DEFAULT_AXIS_SIGN,
		BNO055_NORMAL_MODE,
		IMU,
		CLOCK_EXTERNAL,
		Range_2G
};

float BNO055_Yaw;   // 오프셋을 뺀 현재 방향 (0~360도)
float Delta_Yaw;    // 목표 방향 - 현재 방향 (-180~180도)
float Yaw_Offset;   // 센서 원래 값과 AGV가 쓰는 방향의 차이. 방향 = 센서 원래 값 - 오프셋
PID_t FORWARD_YAW, ROTATE_YAW;   // 직진 중 방향 유지 / 제자리 회전

/* ---- 라즈베리파이 통신 ---- */
uint8_t ArUcoMarker_RxBuf[RPI_RxBuf_SIZE];   // DMA 수신 버퍼 (21바이트 프레임)
RPI_Data_t rpi_data;                          // 받은 데이터를 숫자로 바꾼 결과 (명령, x, y, 각도)
AGV_Event_t evt;                              // 라즈베리파이에 보낼 신호 (ACK: 명령 받음 / DONE: 동작 끝)

/* ---- 리프트 ---- */
volatile uint8_t lift_state = 0;		// UP: 1, DOWN: 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Yaw_Filter_Reset(void);
float ReadYawData(void);

/* printf 출력을 디버그용 USART3로 보낸다 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

/* 플래시에 저장해둔 BNO055 캘리브레이션 값(22바이트)을 읽어온다 */
void Calib_Load(uint8_t *offset)
{
    for (int i = 0; i < 22; i++) {
        // 플래시도 메모리 주소로 접근할 수 있어서, 주소를 포인터로 바꿔 바로 읽는다
        offset[i] = *(uint8_t *)(CALIB_ADDR + i);
    }
}

/* 현재 센서 방향을 yaw 0도로 설정한다 (부팅 시 1회) */
void Set_Zero_Yaw(void)
{
	BNO055_Sensors_t bno055_sensor_data;

	ReadData(&bno055_sensor_data, SENSOR_EULER);

	Yaw_Offset = bno055_sensor_data.Euler.X;

	Yaw_Filter_Reset();
}

/* 마커로 측정한 각도(marker_yaw)를 현재 yaw로 설정한다.
 * 주행하며 쌓인 IMU 오차를 마커 위에서 없애기 위해 사용한다. */
void Set_Marker_Yaw(float marker_yaw)
{
	float filtered_yaw, raw_equiv;

	// 센서 값을 직접 읽으면 튄 값이 섞일 수 있어서, 필터를 거친 yaw를 사용한다
	filtered_yaw = ReadYawData();

	// ReadYawData()는 오프셋을 뺀 값이므로, 다시 더해서 센서 원래 값으로 되돌린다
	raw_equiv = filtered_yaw + Yaw_Offset;

	if (raw_equiv < 0.0f)    raw_equiv += 360.0f;
	if (raw_equiv >= 360.0f) raw_equiv -= 360.0f;

	// 센서 원래 값에서 새 오프셋을 빼면 marker_yaw가 나오도록 오프셋을 다시 계산
	Yaw_Offset = raw_equiv - marker_yaw;

	if (Yaw_Offset < 0.0f)    Yaw_Offset += 360.0f;
	if (Yaw_Offset >= 360.0f) Yaw_Offset -= 360.0f;

	// 오프셋을 바꿔서 yaw가 갑자기 변하므로, 이걸 튄 값으로 착각하지 않게 필터 초기화
	Yaw_Filter_Reset();
}

/* 모터, 엔코더, BNO055, 리프트 초기화 */
void AGV_START(void)
{
	uint8_t offset[22];

	printf("--- AGV Main Program Start ---\r\n");

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			// MOTOR_R_PWM
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);			// MOTOR_L_PWM

	// 실제 배선: TIM3 = 왼쪽 엔코더(PB4, PB5), TIM4 = 오른쪽 엔코더(PB6, PD13)
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  	// MOTOR_L_ENCODER
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  	// MOTOR_R_ENCODER

	BNO055_Init(bno055_init);

	// 저장된 캘리브레이션 값이 있으면 적용 (아무것도 안 쓴 플래시는 0xFF로 읽힘)
	Calib_Load(offset);

	if (offset[0] != 0xFF)
	{
	    printf("Saved Calibration Data found! Applying to BNO055...\r\n");

	    setSensorOffsets(offset);

	    printf("Calibration Applied successfully.\r\n");
	}
	else
	{
	    printf("WARNING: No Calibration Data! Sensor may drift.\r\n");
	    printf("Please run the Calibration Tool firmware first.\r\n");
	}

	HAL_Delay(500);   // 센서 안정화 대기

	Set_Zero_Yaw();

	// 스텝 드라이버 EN 핀 설정
	HAL_GPIO_WritePin(STEP_EN_GPIO_Port, STEP_EN_Pin, GPIO_PIN_SET);

	// TIM5는 TIM2가 내보내는 스텝 펄스 개수를 센다 (1050개가 되면 인터럽트)
	// 초기화할 때 인터럽트 플래그가 미리 켜져 있어서, 지우지 않으면 시작하자마자 인터럽트가 걸린다
	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim5);
}

/* 10 ms 동안의 엔코더 변화량(= 속도)을 구한다 */
// TIM3 = 왼쪽, TIM4 = 오른쪽 엔코더
void ENCODER_COUNT_UPDATE(void)
{
	EncoderL_CurrCount = __HAL_TIM_GET_COUNTER(&htim3);
	EncoderR_CurrCount = __HAL_TIM_GET_COUNTER(&htim4);

	// 카운터가 65535 다음에 0으로 돌아가도, int16_t로 바꿔서 빼면 변화량이 맞게 나온다
	// 방향은 DIR 핀으로 따로 정하므로 크기(절댓값)만 사용
	EncoderL_DeltaCount = ABS((int16_t)(EncoderL_CurrCount - EncoderL_PrevCount));
	EncoderR_DeltaCount = ABS((int16_t)(EncoderR_CurrCount - EncoderR_PrevCount));

	EncoderL_PrevCount = EncoderL_CurrCount;
	EncoderR_PrevCount = EncoderR_CurrCount;
}

/* ---------------------------------------------------------------
 * yaw 튐(글리치) 필터
 * yaw를 읽을 때마다(10 ms마다) 직전 정상값과 비교한다.
 *  - 차이가 10도 미만: 정상값이므로 그대로 사용
 *  - 차이가 10도 이상: 튄 값이므로 버리고, 대신 직전 정상값을 넘겨준다
 *  - 10도 이상이 5번 연속: 오프셋을 다시 맞춰서 지금 들어온 값이 직전 정상값으로 보이게 한다
 *    (예: 90도였는데 152도가 계속 들어오면, 오프셋을 바꿔서 152도가 90도로 보이게)
 * 10 ms 만에 10도를 돈다는 건 초당 1000도라 AGV로는 불가능해서 기준을 10도로 잡았다.
 * --------------------------------------------------------------- */
#define YAW_JUMP_LIMIT_DEG 10.0f      // 직전 정상값과 이만큼 이상 차이 나면 튄 값
#define YAW_GLITCH_MAX_REJECT 5       // 튄 값이 이 횟수만큼 연속되면 오프셋을 다시 맞춤

static float Last_Valid_Yaw = 0.0f;         // 마지막으로 통과한 정상 yaw
static uint8_t Yaw_Filter_Reset_Flag = 1;   // 1이면 비교 없이 다음 값을 정상값으로 받아들임 (비교할 직전값이 없을 때)
static uint8_t Yaw_Glitch_Reject_Cnt = 0;   // 튄 값이 연속으로 나온 횟수

/* 오프셋을 일부러 바꾼 직후에 호출한다.
 * yaw가 갑자기 바뀌어도 튄 값으로 착각하지 않고 새 기준으로 받아들이게 한다 */
static void Yaw_Filter_Reset(void)
{
	Yaw_Filter_Reset_Flag = 1;
	Yaw_Glitch_Reject_Cnt = 0;
}

/* 센서 값에서 오프셋을 뺀 yaw(0~360도)를 돌려준다. 튄 값이면 직전 정상값을 대신 돌려준다 */
float ReadYawData(void)
{
	BNO055_Sensors_t bno055_sensor_data;
	float yaw, diff;

	ReadData(&bno055_sensor_data, SENSOR_EULER);
	yaw = bno055_sensor_data.Euler.X - Yaw_Offset;

	// 0~360도 범위로 맞추기 (예: -20도 → 340도)
	if (yaw < 0.0f)
		yaw += 360.0f;
	else if (yaw >= 360.0f)
		yaw -= 360.0f;

	if (!Yaw_Filter_Reset_Flag)
	{
		// 직전 정상값과의 차이
		// 359도 → 1도처럼 경계를 넘을 때 -358도가 아니라 2도로 계산되게 맞춘다
		diff = yaw - Last_Valid_Yaw;
		if (diff > 180.0f) diff -= 360.0f;
		if (diff < -180.0f) diff += 360.0f;

		if (ABS(diff) > YAW_JUMP_LIMIT_DEG)
		{
			Yaw_Glitch_Reject_Cnt++;

			// 1~4번째 연속: 튄 값은 버리고 직전 정상값을 넘겨준다
			if (Yaw_Glitch_Reject_Cnt < YAW_GLITCH_MAX_REJECT)
			{
				printf("[YAW GLITCH] raw: %4.1f, last: %4.1f -> ignored (%d/%d)\r\n",
				       yaw, Last_Valid_Yaw, Yaw_Glitch_Reject_Cnt, YAW_GLITCH_MAX_REJECT);
				return Last_Valid_Yaw;
			}

			// 5번째 연속: 계속 버리기만 하면 yaw가 한 값에 멈춰버리므로,
			// 지금 센서 값이 직전 정상값으로 보이도록 오프셋을 다시 맞춘다 (이번에도 직전 정상값을 넘겨줌)
			Yaw_Offset = bno055_sensor_data.Euler.X - Last_Valid_Yaw;
			if (Yaw_Offset < 0.0f)    Yaw_Offset += 360.0f;
			if (Yaw_Offset >= 360.0f) Yaw_Offset -= 360.0f;

			printf("[YAW GLITCH] %d회 연속 거부 -> Yaw_Offset 재정렬, 직전값(%4.1f) 유지\r\n",
			       Yaw_Glitch_Reject_Cnt, Last_Valid_Yaw);

			Yaw_Glitch_Reject_Cnt = 0;
			return Last_Valid_Yaw;
		}
	}

	// 정상값: 연속 횟수를 0으로 되돌리고, 이 값을 새 직전 정상값으로 저장
	Yaw_Glitch_Reject_Cnt = 0;
	Yaw_Filter_Reset_Flag = 0;
	Last_Valid_Yaw = yaw;

	return yaw;
}

/* ---------------------------------------------------------------
 * PID 제어 함수 (모두 10 ms 주기로 호출)
 * --------------------------------------------------------------- */
// 좌측 모터 편차 보정용으로 정의했으나 현재 미사용
#define MOTOR_L_BIAS_RATIO 0.011f
#define MOTOR_L_ROTATE_OFFSET 0.45f

/* 좌우 바퀴 속도 제어 (target: 10 ms당 목표 엔코더 카운트) */
void SPEED_PID_Control(float target, uint8_t is_forward)
{
	// 직진/회전에 따라 좌측 목표를 다르게 주던 구조만 남아 있고,
	// 현재는 두 경우 모두 같은 목표를 사용한다 (is_forward 사실상 미사용)
	if (is_forward)
		PID_SetTarget(&MOTOR_L, target);
	else
		PID_SetTarget(&MOTOR_L, target);
	PID_SetTarget(&MOTOR_R, target);

	PID_Compute(&MOTOR_L, EncoderL_DeltaCount);
	PID_Compute(&MOTOR_R, EncoderR_DeltaCount);
}

/* 이동 거리 제어 (target: 목표 거리 mm). Kp가 1이라 출력 = 남은 거리 */
void DISTANCE_PID_Control(float target)
{
	EncoderL_TotalCount += EncoderL_DeltaCount;
	EncoderR_TotalCount += EncoderR_DeltaCount;

	// 좌우 바퀴 이동량이 조금씩 달라서 평균값을 로봇 중심의 이동 거리로 사용
	Encoder_TotalCount = (EncoderL_TotalCount + EncoderR_TotalCount) / 2;

	// 카운트 → 바퀴가 몇 바퀴 돌았는지 → 이동 거리(mm)
	Total_Distance = ((float)Encoder_TotalCount / ENCODER_CPR) * WHEEL_CIRCUMFERENCE;

	PID_SetTarget(&DISTANCE, target);

	PID_Compute(&DISTANCE, Total_Distance);
}

/* 직진 중 방향 유지: 목표 방향에서 벗어난 만큼 되돌리는 보정값 계산 */
void FORWARD_YAW_PID_Control(void)
{
	BNO055_Yaw = ReadYawData();
	Delta_Yaw = Target_Yaw - BNO055_Yaw;

	// 오차를 -180~180도로 맞춤 (예: 350도 차이 → -10도, 더 가까운 쪽으로 돌도록)
	if (Delta_Yaw > 180.0f) Delta_Yaw -= 360.0f;
	if (Delta_Yaw < -180.0f) Delta_Yaw += 360.0f;
	//printf("Delta_Yaw: %f\r\n", Delta_Yaw);

	PID_SetTarget(&FORWARD_YAW, 0.0f);

	PID_Compute(&FORWARD_YAW, Delta_Yaw);
}

/* 제자리 회전: 오차가 +인지 -인지로 도는 방향을 정하고, 오차 크기로 얼마나 세게 돌지 계산 */
void ROTATE_YAW_PID_Control(void)
{
	BNO055_Yaw = ReadYawData();
	Delta_Yaw = Target_Yaw - BNO055_Yaw;

	// 오차를 -180~180도로 맞춤
	if (Delta_Yaw > 180.0f) Delta_Yaw -= 360.0f;
	if (Delta_Yaw < -180.0f) Delta_Yaw += 360.0f;

	// 좌우 바퀴를 서로 반대로 돌려 제자리에서 회전. 오차가 +면 한쪽, -면 반대쪽으로
	if (Delta_Yaw >= 0.0f)
	{
		HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
	}

	// 방향은 위에서 정했으니 오차의 크기만 넣는다.
	// 목표 0에서 |오차|를 빼니까 출력은 항상 음수 → 쓰는 쪽에서 ABS()로 양수로 바꿔 쓴다
	PID_SetTarget(&ROTATE_YAW, 0.0f);

	PID_Compute(&ROTATE_YAW, ABS(Delta_Yaw));
}

/* ---------------------------------------------------------------
 * 동작 함수 (메인 루프가 계속 호출하지만, 실제 계산은 control_flag가 켜지는 10 ms마다 1번)
 * --------------------------------------------------------------- */

/* FORWARD 1단계: 직진 전에 제자리 회전으로 Target_Yaw 방향을 맞춘다
 * (처음에는 달리면서 방향을 보정했는데, 그러면 경로가 휘어서 좌우 오차(x_mm)가
 *  구간마다 점점 커졌다. 그래서 출발 전에 제자리에서 방향부터 맞추도록 바꿈)
 * basePwm: 기본 PWM, target: 기본 목표 속도 (10 ms당 카운트) */
void AGV_ALIGN(uint16_t basePwm, float target)
{
	float pwm, count;
	uint16_t base = basePwm;
	static uint8_t n = 0;   // 오차 1도 이내 연속 횟수

	if (control_flag)
	{
		control_flag = 0;
		print_flag++;

		ENCODER_COUNT_UPDATE();

		// 회전 PID (Target_Yaw로 정렬)
		ROTATE_YAW_PID_Control();

		// 목표에 가까워지면 PWM을 낮춘다: 목표를 지나쳐 버리는 것과 차체 진동을 막기 위해 (듀티 상한과 같이 적용)
		if (ABS(Delta_Yaw) < 5.0f) base = basePwm * 0.6f;
		else if (ABS(Delta_Yaw) < 10.0f) base = basePwm * 0.8f;

		// 회전 PID 결과를 PWM과 목표 속도에 더함 (1.5, 50은 실험으로 맞춘 값)
		pwm = base + (ABS(ROTATE_YAW.pidOutput) * 1.5f);
		count = target + ((ABS(ROTATE_YAW.pidOutput) * 1.5f) / 50.0f);

		// 좌우 바퀴 속도 PID
		SPEED_PID_Control(count, 0);

		MotorL_Speed = pwm + MOTOR_L.pidOutput;
		MotorR_Speed = pwm + MOTOR_R.pidOutput;

		// 최대 듀티 제한 (ARR 4800의 45%): 회전이 너무 격해 차체 진동이 심해서 출력 상한을 둠
		if (MotorL_Speed > 2160) MotorL_Speed = 2160;
		if (MotorL_Speed < 0) MotorL_Speed = 0;
		if (MotorR_Speed > 2160) MotorR_Speed = 2160;
		if (MotorR_Speed < 0) MotorR_Speed = 0;

		// 오차 1도 이내가 10회(100 ms) 연속 유지되면 정렬 완료
		// (0도에 딱 맞추려 하면 조금 차이 날 때 좌우로 왔다 갔다 떨림.
		//  1도 안이면 멈추고, 남은 작은 오차는 직진하면서 방향 PID가 보정)
		if (ABS(Delta_Yaw) <= 1.0f)
		{
			n++;
			MotorL_Speed = 0;   // 1도 안에 들어오면 일단 멈추고 그대로 유지되는지 지켜봄
			MotorR_Speed = 0;

			if (n >= 10)
			{
				n = 0;

				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

				// 정렬 완료: 직진 단계로 넘어가기 전 PID와 엔코더 누적값 초기화
				// (초기화를 안 하면 이전 직진 때 쌓인 I항 누적값과 이전 오차가 남아서,
				//  새 직진을 시작하자마자 엉뚱한 방향으로 보정하는 문제가 있었음)
				PID_Reset(&FORWARD_YAW);
				PID_Reset(&MOTOR_L);
				PID_Reset(&MOTOR_R);
				PID_Reset(&DISTANCE);

				EncoderL_PrevCount = __HAL_TIM_GET_COUNTER(&htim3);
				EncoderR_PrevCount = __HAL_TIM_GET_COUNTER(&htim4);
				EncoderL_TotalCount = 0;
				EncoderR_TotalCount = 0;
				Encoder_TotalCount = 0;
				Total_Distance = 0.0f;

				forward_align_done = 1;

				return;
			}
		}
		else
		{
		    n = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MotorL_Speed);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorR_Speed);

		// 디버그 출력 (100 ms마다)
		if (print_flag >= 10)
		{
		    print_flag = 0;
		    printf("[ALIGN] target: %4.1f, yaw: %4.1f, L_spd: %d, R_spd: %d, L_enc: %d, R_enc: %d\r\n",
		           Target_Yaw, BNO055_Yaw, MotorL_Speed, MotorR_Speed,
		           EncoderL_DeltaCount, EncoderR_DeltaCount);
		}
	}
}

/* FORWARD 2단계: 거리·속도·방향 제어를 합쳐 target_distance만큼 직진
 * target_distance: 목표 거리 (mm), basePwm: 기본 PWM
 * target: 현재 미사용 (목표 속도는 pwm / 50으로 계산) */
void AGV_FORWARD(float target_distance, uint16_t basePwm, float target)
{
	float pwm, count;

	// 양쪽 모터 전진 방향
	HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);

	if (control_flag)
	{
		control_flag = 0;
		print_flag++;

		ENCODER_COUNT_UPDATE();

		// 거리 PID (출력 = 남은 거리)
		DISTANCE_PID_Control(target_distance);

		// 이동한 거리에 따라 속도를 바꾼다: 처음 30%는 점점 빠르게, 중간은 일정하게, 마지막 30%는 점점 느리게
		// (pwm / 50 = 그 PWM에 맞는 목표 엔코더 카운트, 실험으로 맞춘 값)
		if (Total_Distance >= 0 && Total_Distance <= (target_distance * 0.3f))
		{
			// 0~30%: 가속. 목표 - 남은 거리 = 지금까지 이동한 거리, 많이 갈수록 PWM 증가
			pwm = basePwm + ((target_distance - ABS(DISTANCE.pidOutput)) * 2.5f);
			count = pwm / 50.0f;
		}
		else if (Total_Distance > (target_distance * 0.3f) && Total_Distance < (target_distance * 0.7f))
		{
			// 30~70%: 일정한 속도. 가속이 끝난 지점의 PWM 그대로 유지
			pwm = basePwm + (target_distance * 0.3f) * 2.5f;
		    count = pwm / 50.0f;
		}
		else if (Total_Distance >= (target_distance * 0.7f) && Total_Distance <= target_distance)
		{
			// 70~100%: 감속. 남은 거리가 줄어들수록 PWM 감소
			pwm = basePwm + (ABS(DISTANCE.pidOutput) * 2.5f);
			count = pwm / 50.0f;
		}
		else if (Total_Distance >= target_distance)
		{
			// 목표 거리 도달: 정지 후 DONE 전송, 대기 상태로 전환
			printf("Distance: %4.1f\r\n", Total_Distance);
			Total_Distance = 0.0f;
			EncoderL_TotalCount = 0;
			EncoderR_TotalCount = 0;
			Encoder_TotalCount = 0;

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

			evt = EVT_DONE;
			Send_Event(&huart2, evt);
			state = STATE_READY;

			return;
		}

		// 좌우 바퀴 속도 PID
		SPEED_PID_Control(count, 1);

		// 방향 PID
		FORWARD_YAW_PID_Control();

		// 방향 보정값을 한쪽에는 빼고 한쪽에는 더해서, 좌우 속도 차이로 방향을 틀어준다
		MotorL_Speed = pwm + MOTOR_L.pidOutput - FORWARD_YAW.pidOutput;
		MotorR_Speed = pwm + MOTOR_R.pidOutput + FORWARD_YAW.pidOutput;

		//printf("foya: %f\r\n", FORWARD_YAW.pidOutput);

		// 최대 듀티 제한 (ARR 4800의 45%): 회전이 너무 격해 차체 진동이 심해서 출력 상한을 둠
		if (MotorL_Speed > 2160) MotorL_Speed = 2160;
		if (MotorL_Speed < 0) MotorL_Speed = 0;
		if (MotorR_Speed > 2160) MotorR_Speed = 2160;
		if (MotorR_Speed < 0) MotorR_Speed = 0;

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MotorL_Speed);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorR_Speed);

		// 디버그 출력 (100 ms마다)
		if (print_flag >= 10)
		{
		    print_flag = 0;
		    printf("target: %4.1f, yaw: %4.1f, L_spd: %d, R_spd: %d, L_enc: %d, R_enc: %d\r\n",
		           Target_Yaw, BNO055_Yaw, MotorL_Speed, MotorR_Speed,
		           EncoderL_DeltaCount, EncoderR_DeltaCount);
		    printf("Distance: %4.1f\r\n", Total_Distance);
		}
	}
}

/* 양쪽 모터 정지 */
void AGV_STOP(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

/* 리프트 상승: TIM2를 켜면 스텝 펄스가 나가고, TIM5가 펄스 1050개를 세면 인터럽트에서 멈춘다
 * lift_state로 이미 올라가 있으면 다시 올리지 않는다 */
void AGV_LIFT_UP(void)
{
	if (!lift_state)
	{
		HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		lift_state = 1;
	}
}

/* 리프트 하강 (동작 방식은 AGV_LIFT_UP과 같고 방향만 반대) */
void AGV_LIFT_DOWN(void)
{
	if (lift_state)
	{
		HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, GPIO_PIN_SET);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		lift_state = 0;
	}
}

/* 제자리 회전: Target_Yaw에 도달하면 DONE 전송 (구조는 AGV_ALIGN과 같음)
 * basePwm: 기본 PWM, target: 기본 목표 속도 (10 ms당 카운트) */
void AGV_ROTATE(uint16_t basePwm, float target)
{
	float pwm, count;
	uint16_t base = basePwm;
	static uint8_t n = 0;   // 오차 1도 이내 연속 횟수

	if (control_flag)
	{
		control_flag = 0;
		print_flag++;

		ENCODER_COUNT_UPDATE();

		// 회전 PID
		ROTATE_YAW_PID_Control();

		// 목표에 가까워지면 PWM을 낮춘다: 목표를 지나쳐 버리는 것과 차체 진동을 막기 위해 (듀티 상한과 같이 적용)
		if (ABS(Delta_Yaw) < 5.0f) base = basePwm * 0.6f;
		else if (ABS(Delta_Yaw) < 10.0f) base = basePwm * 0.8f;

		// 회전 PID 결과를 PWM과 목표 속도에 더함 (1.5, 50은 실험으로 맞춘 값)
		pwm = base + (ABS(ROTATE_YAW.pidOutput) * 1.5f);
		count = target + ((ABS(ROTATE_YAW.pidOutput) * 1.5f) / 50.0f);

		// 좌우 바퀴 속도 PID
		SPEED_PID_Control(count, 0);

		MotorL_Speed = pwm + MOTOR_L.pidOutput;
		MotorR_Speed = pwm + MOTOR_R.pidOutput;

		// 최대 듀티 제한 (ARR 4800의 45%): 회전이 너무 격해 차체 진동이 심해서 출력 상한을 둠
		if (MotorL_Speed > 2160) MotorL_Speed = 2160;
		if (MotorL_Speed < 0) MotorL_Speed = 0;
		if (MotorR_Speed > 2160) MotorR_Speed = 2160;
		if (MotorR_Speed < 0) MotorR_Speed = 0;

		// 오차 1도 이내가 10회(100 ms) 연속 유지되면 회전 완료
		if (ABS(Delta_Yaw) <= 1.0f)
		{
			n++;
			MotorL_Speed = 0;
			MotorR_Speed = 0;

			if (n >= 10)
			{
				n = 0;
				evt = EVT_DONE;
				Send_Event(&huart2, evt);
				state = STATE_READY;
			}
		}
		else
		{
		    n = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MotorL_Speed);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorR_Speed);

		// 디버그 출력 (100 ms마다)
		if (print_flag >= 10)
		{
		    print_flag = 0;
		    printf("pidOutput: %4.1f, target: %4.1f\r\n", ROTATE_YAW.pidOutput, count);
		    printf("target: %4.1f, yaw: %4.1f, L_spd: %d, R_spd: %d, L_enc: %d, R_enc: %d\r\n",
		           Target_Yaw, BNO055_Yaw, MotorL_Speed, MotorR_Speed,
		           EncoderL_DeltaCount, EncoderR_DeltaCount);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // 시스템 클럭 96 MHz: 외부 8 MHz(ST-LINK가 공급) ÷ 4 × 96 ÷ 2
  // APB1 48 MHz (여기 붙은 타이머는 2배인 96 MHz), APB2 96 MHz

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  /* PID 초기화: Kp, Ki, Kd, dt(10 ms), 초기 목표, 출력 상한, 하한 */
  PID_Init(&MOTOR_L, 1.0, 0.0, 0.0, 0.01, 0, 4320, -4320);
  PID_Init(&MOTOR_R, 1.0, 0.0, 0.0, 0.01, 0, 4320, -4320);
  PID_Init(&FORWARD_YAW, 30.0, 0.005, 0.055, 0.01, 0, 4320, -4320);
  PID_Init(&ROTATE_YAW, 5.0, 0.0001, 0.1, 0.01, 0, 4320, -4320);
  PID_Init(&DISTANCE, 1.0, 0.0, 0.0, 0.01, 0, 4320, -4320);

  HAL_TIM_Base_Start_IT(&htim6);   // 10 ms 제어 주기 시작

  // 라즈베리파이 데이터 받기 시작: DMA가 알아서 받다가, 데이터가 끊기면(한 프레임 끝) 콜백 호출
  // 절반 받았을 때 오는 인터럽트는 필요 없어서 끈다
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ArUcoMarker_RxBuf, RPI_RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  AGV_START();

  // 출발할 때 방향을 90도로 정함 (base_angle 시작값과 같게)
  Set_Marker_Yaw(90.0f);

  // 준비 완료를 라즈베리파이에 알림
  Send_Event(&huart2, EVT_DONE);

  float correct_angle = 0.0f,        // 좌우로 밀린 만큼 틀어줄 각도 (deg)
        x_mm = 0.0f, y_mm = 0.0f,    // 마커 중심 기준 AGV 위치 (mm)
        base_angle = 90.0f;          // AGV가 지금 바라봐야 하는 방향 (회전 명령마다 90도나 180도씩 바뀜)
  float marker_yaw = 0.0f,           // 마커로 측정한 AGV 방향 (deg)
        dyaw = 0.0f;                 // 카메라로 잰 방향 - 바라봐야 하는 방향
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* ---- 동작 중: 명령에 맞는 동작 함수를 반복 호출 ---- */
	  if (state == STATE_MOVE)
	  {
		  switch(command)
		  {
		  case CMD_FORWARD:
			  // 1) 아직 Target_Yaw로 정렬 안 됐으면 제자리 회전으로 먼저 정렬
			  // 2) 방향을 맞춘 다음에 직진 (비스듬히 출발하면 휘어서 가므로)
			  if (!forward_align_done)
				  AGV_ALIGN(1000, 20.0f);
			  else
				  AGV_FORWARD(Target_Distance, 1200, 24.0f);
			  break;
		  case CMD_STOP:
			  // 주의: STOP은 DONE을 보내지 않아 MOVE 상태에 머문다
			  AGV_STOP();
			  break;
		  case CMD_LIFT_UP:
			  // 완료는 TIM5 콜백 -> lift_done_flag로 처리
			  // 주의: 이미 올라간 상태에서 받으면 펄스가 나가지 않아 DONE도 전송되지 않는다
			  AGV_LIFT_UP();
			  break;
		  case CMD_LIFT_DOWN:
			  AGV_LIFT_DOWN();
			  break;
		  case CMD_ROTATE_LEFT:
			  AGV_ROTATE(1000, 20.0f);
			  break;
		  case CMD_ROTATE_RIGHT:
			  AGV_ROTATE(1000, 20.0f);
			  break;
		  case CMD_ROTATE_180:
			  AGV_ROTATE(1000, 20.0f);
			  break;
		  default:
			  AGV_STOP();
			  break;
		  }
	  }
	  /* ---- 대기 중: 프레임을 받으면 목표값을 계산하고 MOVE로 전환 ---- */
	  else if (state == STATE_READY)
	  {
		  if (usart2_flag)
		  {
			  usart2_flag = 0;
			  Read_RPI_Data(&rpi_data, ArUcoMarker_RxBuf);

			  // 소수점 한 자리까지 보내려고 10을 곱해서 정수로 보내므로 10으로 나눈다
			  // (처음엔 실수 문자열을 atof()로 변환했는데, 직접 만든 정수 파싱으로 바꿈.
			  //  자릿수가 고정되어 21바이트 길이 검사가 가능하고, 변환도 단순하고 빠름)
			  command = rpi_data.command;
			  x_mm = rpi_data.x_offset / 10.0f;
			  y_mm = rpi_data.y_offset / 10.0f;
			  marker_yaw = (rpi_data.yaw_offset / 10.0f);

			  // 마커로 측정한 방향과 기준 방향의 차이 (-180~180)
			  dyaw = marker_yaw - base_angle;

			  if (dyaw > 180.0f) dyaw -= 360.0f;
			  if (dyaw < -180.0f) dyaw += 360.0f;

			  // 범위를 벗어난 값은 깨진 값으로 보고 무시한다.
			  // 데이터를 통째로 버리면 ACK를 못 보내서 라즈베리파이가 계속 기다리므로, 명령은 그대로 실행
			  if (ABS(x_mm) > MARKER_X_MM_LIMIT || ABS(y_mm) > MARKER_Y_MM_LIMIT)
			  {
				  printf("[MARKER GLITCH] x_mm=%.2f, y_mm=%.2f out of range (x>%.0f or y>%.0fmm), offset zeroed\r\n",
						  x_mm, y_mm, MARKER_X_MM_LIMIT, MARKER_Y_MM_LIMIT);
				  x_mm = 0.0f;
				  y_mm = 0.0f;
			  }

			  if (command >= CMD_FORWARD && command <= CMD_ROTATE_180)
			  {
				  // 카메라로 잰 방향이 바라봐야 하는 방향과 0.5도 안으로 맞으면, IMU yaw를 그 방향으로 다시 맞춘다
				  // (주행하면서 IMU에 조금씩 쌓인 오차를 없앰)
				  if (ABS(dyaw) <= 0.5f)
				  {
					  printf("UPDATE YAW OFFSET!!!!!\r\n");
					  Set_Marker_Yaw(base_angle);
				  }

				  if (command == CMD_FORWARD)
				  {
					  // 지금 위치에서 다음 마커까지 x, y 거리 (TARGET_DISTANCE 기준)
					  float dx = -x_mm;
					  float dy = TARGET_DISTANCE - y_mm;

					  // 좌우로 5 mm 이상 밀렸을 때만, 다음 마커 쪽으로 각도를 틀어준다
					  if (ABS(x_mm) >= 5.0f)
						  correct_angle = atan2f(dx, dy) * 180.0f / M_PI;
					  else
						  correct_angle = 0.0f;

					  // 보정 각도는 ±5도로 제한
					  if (correct_angle > 5.0f)  correct_angle = 5.0f;
					  if (correct_angle < -5.0f) correct_angle = -5.0f;

					  Target_Yaw = base_angle + correct_angle;

					  // 비스듬히 가면 거리가 조금 늘어나므로 다시 계산
					  Target_Distance = sqrtf(dx * dx + dy * dy);
					  if (Target_Yaw >= 360.0f) Target_Yaw -= 360.0f;
					  if (Target_Yaw < 0.0f)    Target_Yaw += 360.0f;
					  printf("\r\nx_mm: %.2f, y_mm: %.2f, ang: %.2f, dist: %.2f, base: %.1f\r\n",
							  x_mm, y_mm, Target_Yaw, Target_Distance, base_angle);

					  PID_Reset(&FORWARD_YAW);
					  PID_Reset(&ROTATE_YAW);

					  forward_align_done = 0;   // 방향 맞추기부터 시작
				  }

				  if (command == CMD_ROTATE_LEFT)
				  {
					  // 기준 방향을 왼쪽으로 90도 변경
					  base_angle -= 90.0f;
					  if (base_angle >= 360.0f) base_angle -= 360.0f;
					  if (base_angle < 0.0f)    base_angle += 360.0f;

					  Target_Yaw = base_angle;

					  // 마커로 측정한 방향 오차가 5~10도면 그만큼 목표를 보정
					  // 10도 이상이면 마커를 잘못 읽은 걸로 보고 보정 안 함
					  if (ABS(dyaw) >= 5.0f && ABS(dyaw) < MARKER_DYAW_LIMIT_DEG)
					  {
						  Target_Yaw -= dyaw;

						  if (Target_Yaw < 0.0f)    Target_Yaw += 360.0f;
						  if (Target_Yaw >= 360.0f) Target_Yaw -= 360.0f;
					  }
					  else if (ABS(dyaw) >= MARKER_DYAW_LIMIT_DEG)
					  {
						  printf("[MARKER GLITCH] dyaw=%.2f out of range (>=%.0fdeg), correction skipped\r\n",
						         dyaw, MARKER_DYAW_LIMIT_DEG);
					  }

					  printf("\r\nx_mm: %.2f, y_mm: %.2f, dyaw: %.2f, ang: %.2f\r\n",
							  x_mm, y_mm, dyaw, Target_Yaw);

					  PID_Reset(&ROTATE_YAW);
				  }

				  if (command == CMD_ROTATE_RIGHT)
				  {
					  // 기준 방향을 오른쪽으로 90도 변경 (보정 방식은 ROTATE_LEFT와 같음)
					  base_angle += 90.0f;
					  if (base_angle >= 360.0f) base_angle -= 360.0f;
					  if (base_angle < 0.0f)    base_angle += 360.0f;

					  Target_Yaw = base_angle;

					  if (ABS(dyaw) >= 5.0f && ABS(dyaw) < MARKER_DYAW_LIMIT_DEG)
					  {
						  Target_Yaw -= dyaw;

						  if (Target_Yaw < 0.0f)    Target_Yaw += 360.0f;
						  if (Target_Yaw >= 360.0f) Target_Yaw -= 360.0f;
					  }
					  else if (ABS(dyaw) >= MARKER_DYAW_LIMIT_DEG)
					  {
						  printf("[MARKER GLITCH] dyaw=%.2f out of range (>=%.0fdeg), correction skipped\r\n",
						         dyaw, MARKER_DYAW_LIMIT_DEG);
					  }

					  printf("\r\nx_mm: %.2f, y_mm: %.2f, dyaw: %.2f, ang: %.2f\r\n",
							  x_mm, y_mm, dyaw, Target_Yaw);

					  PID_Reset(&ROTATE_YAW);
				  }

				  if (command == CMD_ROTATE_180)
				  {
					  // 기준 방향을 180도 변경 (보정 방식은 ROTATE_LEFT와 같음)
					  base_angle += 180.0f;
					  if (base_angle >= 360.0f) base_angle -= 360.0f;
					  if (base_angle < 0.0f)    base_angle += 360.0f;

					  Target_Yaw = base_angle;

					  if (ABS(dyaw) >= 5.0f && ABS(dyaw) < MARKER_DYAW_LIMIT_DEG)
					  {
						  Target_Yaw -= dyaw;

						  if (Target_Yaw < 0.0f)    Target_Yaw += 360.0f;
						  if (Target_Yaw >= 360.0f) Target_Yaw -= 360.0f;
					  }
					  else if (ABS(dyaw) >= MARKER_DYAW_LIMIT_DEG)
					  {
						  printf("[MARKER GLITCH] dyaw=%.2f out of range (>=%.0fdeg), correction skipped\r\n",
						         dyaw, MARKER_DYAW_LIMIT_DEG);
					  }

					  printf("\r\nx_mm: %.2f, y_mm: %.2f, dyaw: %.2f, ang: %.2f\r\n",
							  x_mm, y_mm, dyaw, Target_Yaw);

					  PID_Reset(&ROTATE_YAW);
				  }

				  // 수신 확인(ACK) 전송 후 동작 시작
				  evt = EVT_ACK;

				  // 기다리는 동안 바뀐 엔코더 값이 첫 계산에 섞이지 않도록 지금 값으로 맞춰둠
				  EncoderL_PrevCount = __HAL_TIM_GET_COUNTER(&htim3);
				  EncoderR_PrevCount = __HAL_TIM_GET_COUNTER(&htim4);

				  Send_Event(&huart2, evt);
				  state = STATE_MOVE;
			  }
		  }
	  }

	  // 리프트 목표 스텝 도달 (TIM5 콜백) -> DONE 전송
	  if (lift_done_flag)
	  {
		  lift_done_flag = 0;
		  evt = EVT_DONE;
		  Send_Event(&huart2, evt);
		  state = STATE_READY;
	  }
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  // I2C1: BNO055 통신

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  // TIM1: 오른쪽 DC 모터 PWM. 96 MHz ÷ 4800 = 20 kHz (사람 귀에 안 들리는 주파수라 모터 소리가 줄어듦)

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  // TIM2: 리프트 스텝모터에 펄스를 내보냄. 96 MHz ÷ 96 ÷ 2000 = 500 Hz (1초에 500스텝)
  // 펄스 하나 낼 때마다 TIM5에 신호를 보내서 개수를 세게 한다

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  // TIM3: 왼쪽 바퀴 엔코더. A, B 신호의 모든 변화를 세서 4배로 정밀하게 읽음, 노이즈 필터 최대(15)

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  // TIM4: 오른쪽 바퀴 엔코더. 설정은 TIM3과 동일

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  // TIM5: TIM2가 낸 펄스 개수를 세는 타이머 (슬레이브 모드, TIM2 신호를 클럭으로 사용)
  // 1050개가 되면 인터럽트 → 콜백에서 리프트 정지

  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1050-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  // TIM6: 제어 주기 타이머. 96 MHz ÷ 9600 ÷ 100 = 100 Hz (10 ms마다 인터럽트)

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9600-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  // TIM8: 왼쪽 DC 모터 PWM. 설정은 TIM1과 동일 (20 kHz)

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4800-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  // USART2: 라즈베리파이 통신 (115200 bps, 받는 쪽은 DMA 사용)

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  // USART3: 디버그 출력 (USB로 PC에 연결되는 시리얼, printf가 여기로 나감)

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|MOTOR_L_DIR_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_DIR_Pin|STEP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin MOTOR_L_DIR_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|MOTOR_L_DIR_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_R_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_R_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_DIR_Pin STEP_EN_Pin */
  GPIO_InitStruct.Pin = STEP_DIR_Pin|STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 현재 안 쓰는 함수: USART3로 받기를 시작하는 코드가 없어서 호출되지 않는다.
 * 처음에 PC 터미널로 회전 명령을 테스트하던 코드로 보임 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		if (command == CMD_ROTATE_LEFT)
			Target_Yaw -= 90.0f;
		else if (command == CMD_ROTATE_RIGHT)
			Target_Yaw += 90.0f;
		else if (command == CMD_ROTATE_180)
			Target_Yaw += 180.0f;

		if (Target_Yaw >= 360.0) Target_Yaw -= 360;
		if (Target_Yaw < 0.0) Target_Yaw += 360;
	}
}

/* 타이머 업데이트 인터럽트 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIM6 (10 ms마다): 신호만 켜고, 실제 계산은 메인 루프에서 한다 (인터럽트는 짧게)
    if (htim->Instance == TIM6) {
    	control_flag++;
    }

	// TIM5: 펄스 1050개 다 셈 → 펄스 멈추고, 카운터 0으로, 메인 루프에 리프트 완료 알림
	if (htim->Instance == TIM5)
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        __HAL_TIM_SET_COUNTER(&htim5, 0);

        lift_done_flag++;
	}
}

/* 라즈베리파이 데이터 한 덩어리(프레임)를 다 받으면 호출된다 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		// 길이가 21바이트이고 '<'로 시작해서 '>'로 끝나면 정상
		if (Size == RPI_RxBuf_SIZE && ArUcoMarker_RxBuf[0] == '<' && ArUcoMarker_RxBuf[20] == '>')
		{
			//printf("%s\r\n", ArUcoMarker_RxBuf);
			usart2_flag = 1;
		}
		else
		{
			// 깨진 데이터: 버리고 다시 받기 시작
			printf("RX FAIL sz=%d [0]=%d [20]=%d\r\n",
			       Size, ArUcoMarker_RxBuf[0], ArUcoMarker_RxBuf[20]);
			HAL_UART_DMAStop(&huart2);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ArUcoMarker_RxBuf, RPI_RxBuf_SIZE);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		}
	}
}

/* UART 통신 오류가 나면 호출된다 (데이터를 놓치거나, 노이즈, 형식 오류) */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
    	// 오류 표시를 지우고 다시 받기 시작 (안 하면 받기가 멈춰버림)
    	__HAL_UART_CLEAR_OREFLAG(huart);
    	__HAL_UART_CLEAR_NEFLAG(huart);
    	__HAL_UART_CLEAR_FEFLAG(huart);

		HAL_UART_DMAStop(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ArUcoMarker_RxBuf, RPI_RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
