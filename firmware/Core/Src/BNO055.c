/*
 * BNO055.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Berat Bayram
 */
#include "BNO055_STM32.h"
#include <string.h>

/*!
 *   @brief  Gets the latest system status info
 *
 *   @param  BNO_status_t structure that contains status information
 *           STresult, SYSError and SYSStatus
 *
 *   @retval None
 */
// BNO055 자체 테스트하는 함수
void Check_Status(BNO_Status_t *result){

	HAL_StatusTypeDef status;		// I2C 통신 결과를 저장할 변수 (HAL_OK, HAL_ERROR 등)
	uint8_t value;					// 센서 레지스터에서 읽어올 데이터를 임시 저장하는 변수

	// 1. 셀프 테스트 결과 읽기 -  ST_RESULT 0x36
	  /* Self Test Results
	     1 = test passed, 0 = test failed

	     Bit 0 = Accelerometer self test
	     Bit 1 = Magnetometer self test
	     Bit 2 = Gyroscope self test
	     Bit 3 = MCU self test

	     0x0F = all good!
		*/
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ST_RESULT_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: ST_RESULT_ADDR\r\n");
	}
	HAL_Delay(50);
	result->STresult = value;
	value=0;

	// 2. 시스템 상태 읽기 - SYS_STATUS 0x39
	  /* System Status (see section 4.3.58)
	     0 = Idle
	     1 = System Error
	     2 = Initializing Peripherals
	     3 = System Iniitalization
	     4 = Executing Self-Test
	     5 = Sensor fusio algorithm running
	     6 = System running without fusion algorithms
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_STATUS_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_STATUS_ADDR\r\n");
	}
	HAL_Delay(50);
	result->SYSStatus = value;
	value=0;

	// 3. 시스템 에러 읽기 - SYS_ERR 0x3A
	  /* System Error (see section 4.3.59)
	     0 = No error
	     1 = Peripheral initialization error
	     2 = System initialization error
	     3 = Self test result failed
	     4 = Register map value out of range
	     5 = Register map address out of range
	     6 = Register map write error
	     7 = BNO low power mode not available for selected operation mode
	     8 = Accelerometer power mode not available
	     9 = Fusion algorithm configuration error
	     A = Sensor configuration error
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_ERR_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_ERR_ADDR\r\n");
	}
	HAL_Delay(50);
	result->SYSError = value;
}

/*!
 *   @brief  Changes register page
 *
 *   @param  Page number
 *   		Possible Arguments
 * 			[PAGE_0
 * 			 PAGE_1]
 *
 * 	 @retval None
 */
// BNO055 Page를 설정하는 함수
// Page_0: 읽기, Page_1: 쓰기
void SelectPage(uint8_t page){

	if(HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PAGE_ID_ADDR, 1, &page, 1, 100) != HAL_OK){
		printf("Register page replacement could not be set\r\n");
	}
	HAL_Delay(50);
}

/**
  * @brief  Software Reset to BNO055
  *
  * @param  None
  *
  * @retval None
  */
void ResetBNO055(void){

	uint8_t reset = 0x20;		// 0010 0000: SYS_TRIGGER 레지스터의 리셋 명령 비트
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &reset, 1, 100);
	HAL_Delay(500);

	//Checking for is reset process done
	uint8_t chip_id=0;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, CHIP_ID_ADDR, 1, &chip_id, 1, 100);

	//If value of id register is not equal to BNO055 chip id which is 0xA0, wait until equal to each other
	while(chip_id != BNO055_ID) {
		printf("BNO055-> Undefined chip id\r\n");
		HAL_Delay(500);
		HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, CHIP_ID_ADDR, 1, &chip_id, 1, 100);
	}
}

/*!
 *   @brief  Reads various data measured by BNO055
 *
 *   @param  Register base address of the data to be read
 * 			Possible arguments
 * 			[SENSOR_ACCEL
 *			 SENSOR_GYRO
 * 			 SENSOR_MAG
 *			 SENSOR_EULER
 *			 SENSOR_LINACC
 *			 SENSOR_GRAVITY
 *			 SENSOR_QUATERNION]
 *
 *   @retval Structure containing the values ​​of the read data
 */
void ReadData(BNO055_Sensors_t *sensorData,BNO055_Sensor_Type sensors){


	   uint8_t buffer[8];

	    if (sensors & SENSOR_GRAVITY) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GRAVITY, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gravity.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->Gravity.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Gravity.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_QUATERNION) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_QUATERNION, 1, buffer, 8, HAL_MAX_DELAY);
	        sensorData->Quaternion.W = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/(1<<14));
	        sensorData->Quaternion.X = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/(1<<14));
	        sensorData->Quaternion.Y = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/(1<<14));
	        sensorData->Quaternion.Z = (float)(((int16_t)((buffer[7] << 8) | buffer[6]))/(1<<14));
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_LINACC) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_LINACC, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->LineerAcc.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->LineerAcc.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->LineerAcc.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_GYRO) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GYRO, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gyro.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Gyro.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Gyro.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_ACCEL) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_ACCEL, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Accel.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->Accel.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Accel.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_MAG) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_MAG, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Magneto.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Magneto.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Magneto.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_EULER) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_EULER, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Euler.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Euler.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Euler.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  Operation modes
 *          Mode Values;
 *           [CONFIG_MODE
 *            ACC_ONLY
 *            MAG_ONLY
 *            GYR_ONLY
 *            ACC_MAG
 *            ACC_GYRO
 *            MAG_GYRO
 *            AMG
 *            IMU
 *            COMPASS
 *            M4G
 *            NDOF_FMC_OFF
 *            NDOF]
 *
 *  @retval None
 */
// BNO055 동작 모드를 설정하는 함수
void Set_Operation_Mode(Op_Modes_t Mode){

	SelectPage(PAGE_0);
	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &Mode, 1, 100) !=HAL_OK){
		printf("Operation mode could not be set!\r\n");
	}
	else printf("Operation mode switching succeeded.\r\n");

	if(Mode == CONFIG_MODE) HAL_Delay(19);

	else HAL_Delay(9);

}

/*!
 *  @brief  Set the power mode of BNO055
 *  @param  power modes
 *          possible values
 *           [BNO055_NORMAL_MODE
 *            BNO055_LOWPOWER_MODE
 *            BNO055_SUSPEND_MODE]
 *
 *  @retval None
 */
// BNO055 전원 모드 설정하는 함수
void SetPowerMODE(uint8_t BNO055_){

	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PWR_MODE_ADDR, 1, &BNO055_, 1, 100) != HAL_OK)
	{
		printf("Power mode could not be set!\r\n");
	}
	else
	{
		printf("Power mode switching succeeded.\r\n");
	}
	HAL_Delay(50);

}

/*!
 *  @brief  Selects the chip's clock source
 *  @param  Source
 *          possible values
 *           [CLOCK_EXTERNAL
 *            CLOCK_INTERNAL]
 *
 *  @retval None
 */
// BNO055 클럭 소스 선택하는 함수
void Clock_Source(uint8_t source) {

	//7th bit: External Crystal=1; Internal Crystal=0
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &source, sizeof(source), 100);
}

/*!
 *  @brief  Changes the chip's axis signs and remap
 *  @param  remapcode and signcode
 *         	Default Parameters:[DEFAULT_AXIS_REMAP(0x24), DEFAULT_AXIS_SIGN(0x00)]
 *
 *  @retval None
 */
// BNO055 축 매핑 및 부호 설정하는 함수
void BNO055_Axis(uint8_t remap, uint8_t sign){

	// X, Y, Z 축을 실제 방향에 맞게 재배치
	HAL_I2C_Mem_Write(&bno_i2c,P_BNO055, AXIS_MAP_CONFIG_ADDR, 1, &remap, 1, 100);
	HAL_Delay(20);
	// 부호 설정
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, AXIS_MAP_SIGN_ADDR, 1, &sign, 1, 100);
	HAL_Delay(100);
}

/*!
 *  @brief  Sets the accelerometer range
 *  @param  range
 *          possible values
 *           [Range_2G
 *            Range_4G
 *            Range_8G
 *            Range_16G]
 *
 *  @retval None
 */
// BNO055 가속도계 측정 범위 설정하는 함수
void SET_Accel_Range(uint8_t range){

	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_CONFIG_ADDR, 1, &range, 1, 100);
	HAL_Delay(100);

}

/**
  * @brief  Initialization of BNO055
  *
  * @param  Init argument to a BNO055_Init_t structure that contains
  *         the configuration information for the BNO055 device.
  *
  * @retval None
  */
void BNO055_Init(BNO055_Init_t Init) {

	// 1. CONFIG 모드 진입
	//Set operation mode to config_mode for initialize all register
	Set_Operation_Mode(CONFIG_MODE);
	HAL_Delay(50);

	// 2. 가속도계 범위 설정
	/*
	 * Set register page number to 1
	 * Configure Accelerometer range
	 */
	SelectPage(PAGE_1);
	SET_Accel_Range(Init.ACC_Range);
	HAL_Delay(50);

	// 3. 페이지 복귀
	//Set register page number to 0
	SelectPage(PAGE_0);
	HAL_Delay(50);

	// 4. 클럭 소스 설정
	//Read clock status. If status=0 then it is free to configure the clock source
	uint8_t status;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_CLK_STATUS_ADDR, 1, &status, 1, 100);
	HAL_Delay(50);
	//Checking if the status bit is 0
	if(status == 0)
	{
		//Changing clock source
		Clock_Source(Init.Clock_Source);
		HAL_Delay(100);
	}

	// 5. 축 매핑
	//Configure axis remapping and signing
	BNO055_Axis(Init.Axis, Init.Axis_sign);
	HAL_Delay(100);

	// 6. 단위 설정
	//Configure data output format and the measurement unit
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, UNIT_SEL_ADDR, 1, &Init.Unit_Sel, sizeof(Init.Unit_Sel), 100);
	HAL_Delay(100);

	// 7. 전원 모드 설정
	//Set power mode
	SetPowerMODE(Init.Mode);
	HAL_Delay(100);

	// 8. 동작 모드 설정
	//Set operation mode
	Set_Operation_Mode(Init.OP_Modes);
	HAL_Delay(100);

	printf("BNO055 Initialization process is done!\r\n");
}

/**
  * @brief  Gets calibration status of accel, gyro, mag and system
  *
  * @param  None
  *
  * @retval Calib_status_t structure that contains
  *         the calibration status of accel, gyro, mag and system.
  */
// BNO055 캘리브레이션 상태를 읽는 함수
void getCalibration(Calib_status_t *calib) {
    uint8_t calData;

    // Read calibration status register using I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, CALIB_STAT_ADDR, 1, &calData, 1, HAL_MAX_DELAY);

    // Check if read was successful
    if (status == HAL_OK) {

        // Extract calibration status values

        	calib->System= (calData >> 6) & 0x03;


        	calib->Gyro = (calData >> 4) & 0x03;


        	calib->Acc = (calData >> 2) & 0x03;


        	calib->MAG = calData & 0x03;

    } else {
        printf("Failed to read calibration status register.\r\n");
    }
}

/**
  * @brief  Gets sensor offsets
  *
  * @param  22 byte long buffer to hold offset data
  *
  * @retval None
  *
  */
// BNO055 캘리브레이션 오프셋을 읽는 함수
void getSensorOffsets(uint8_t *calibData) {

        // Save the current mode
        uint8_t lastMode = getCurrentMode();

        // Switch to CONFIG mode
        Set_Operation_Mode(CONFIG_MODE);
        printf("Switched to CONFIG mode.\r\n");

        // Read the offset registers
        HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, calibData, 22, 100);
        printf("Calibration data obtained.\r\n");

        // Restore the previous mode
        Set_Operation_Mode(lastMode);
        printf("Restored to previous mode.\r\n");
}

/**
  * @brief  Sets sensor offsets
  *
  * @param  22 byte long buffer containing offset data
  *
  * @retval None
  *
  */
// BNO055 캘리브레이션 오프셋을 쓰는 함수
void setSensorOffsets(const uint8_t *calibData) {
    uint8_t lastMode = getCurrentMode();

    // Switch to CONFIG mode
    Set_Operation_Mode(CONFIG_MODE);
    printf("Switched to CONFIG mode.\r\n");

    // Write calibration data to the sensor's offset registers using memory write
    HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, (uint8_t *)calibData, 22, 100);
    printf("Wrote calibration data to sensor's offset registers.\r\n");

    // Restore the previous mode
    Set_Operation_Mode(lastMode);
    printf("Restored to previous mode.\r\n");
}

/**
  * @brief  Checks the calibration status of the sensor
  *
  * @param  None
  *
  * @retval True of False
  *
  */
// BNO055 캘리브레이션 완료 확인 하는 함수
bool isFullyCalibrated(void) {
//    Calib_status_t calib ={0};
    Calib_status_t calib ={0};
    getCalibration(&calib);


    switch (getCurrentMode()) {
        case ACC_ONLY:
            return (calib.Acc == 3);
        case MAG_ONLY:
            return (calib.MAG == 3);
        case GYRO_ONLY:
        case M4G: /* No magnetometer calibration required. */
            return (calib.Gyro == 3);
        case ACC_MAG:
        case COMPASS:
            return (calib.Acc == 3 && calib.MAG == 3);
        case ACC_GYRO:
        case IMU:
            return (calib.Acc == 3 && calib.Gyro == 3);
        case MAG_GYRO:
            return (calib.MAG == 3 && calib.Gyro == 3);
        default:
            return (calib.System == 3 && calib.Gyro == 3 && calib.Acc == 3 && calib.MAG == 3);
    }
}

/**
  * @brief  Gets the current operating mode of the chip
  *
  * @param  None
  *
  * @retval Operating mode
  *
  */
 // BNO055 현재 동작 모드를 읽는 함수
Op_Modes_t getCurrentMode(void) {

	Op_Modes_t mode;

	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &mode, 1, 100);

    return mode;
}

/**
  * @brief  Calibrates BNO055
  *
  * @param  None
  *
  * @retval None
  *
  */
// BNO055 전체 캘리브레이션 수행하는 함수
bool Calibrate_BNO055(void) {

		Calib_status_t calib={0};
        printf("Calibrating BNO055 sensor...\r\n");

		// 1. 자이로 캘리브레이션
        // Set operation mode to FUSION_MODE or appropriate mode for calibration
        Set_Operation_Mode(NDOF);
    	HAL_Delay(100);
        // Gyroscope calibration
        printf("Calibrating gyroscope...\r\n");
        printf("Place the device in a single stable position\r\n");
        printf("Starting in 3 seconds...\r\n");
        HAL_Delay(1000);
        printf("2...\r\n");
        HAL_Delay(1000);
        printf("1...\r\n");
        HAL_Delay(1000);

        do {
            getCalibration(&calib);
            printf("Gyro Calib Level: %d / 3\r\n", calib.Gyro);
            HAL_Delay(500); // 0.5초마다 상태 확인
		} while (calib.Gyro !=3);
        printf("Gyroscope calibration complete.\r\n");

		// 2. 가속도계 캘리브레이션
        // Accelerometer calibration
        printf("Calibrating accelerometer...\r\n");
        printf("Place the device in 6 different stable positions\r\n");

        do {
            getCalibration(&calib);
            printf("Accel Calib Level: %d / 3\r\n", calib.Acc);
        	HAL_Delay(500);
		} while (calib.Acc !=3);
        printf("Accelerometer calibration complete.\r\n");

		// 3. 자력계 캘리브레이션
        // Magnetometer calibration
        printf("Calibrating magnetometer...\r\n");
        printf("Make some random movements\r\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
            printf("Mag Calib Level: %d / 3\r\n", calib.MAG);
        	HAL_Delay(500);
		} while (calib.MAG !=3);
        printf("Magnetometer calibration complete.\r\n");

		// 4. 시스템 캘리브레이션
        // System calibration
        printf("Calibrating system...\r\n");
        printf("Keep the device stationary until system calibration reaches level 3\r\n");
        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.System !=3);
        HAL_Delay(500);

        // Check calibration status
        while(!isFullyCalibrated()) HAL_Delay(500);
        printf("Sensor is fully calibrated.\r\n");

        printf("System: %d      Gyro: %d       Accel: %d       MAG: %d\r\n",calib.System,calib.Gyro , calib.Acc, calib.MAG);
        if(isFullyCalibrated()) return true;
        else return false;
}
