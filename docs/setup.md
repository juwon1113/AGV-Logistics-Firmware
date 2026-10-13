# 핀 배치, 빌드, 캘리브레이션

[← README로 돌아가기](../README.md)

## 핀 배치

| 핀 | 기능 | 연결 |
| --- | --- | --- |
| PC6 | TIM8_CH1 | 왼쪽 모터 PWM |
| PB15 | GPIO 출력 | 왼쪽 모터 DIR |
| PE9 | TIM1_CH1 | 오른쪽 모터 PWM |
| PF13 | GPIO 출력 | 오른쪽 모터 DIR |
| PB4, PB5 | TIM3_CH1, CH2 | 왼쪽 엔코더 A, B |
| PB6, PD13 | TIM4_CH1, CH2 | 오른쪽 엔코더 A, B |
| PA0 | TIM2_CH1 | 리프트 스텝 펄스 (STEP) |
| PD14 | GPIO 출력 | 리프트 방향 (DIR) |
| PD15 | GPIO 출력 | 리프트 드라이버 활성화 (EN) |
| PB8, PB9 | I2C1 SCL, SDA | BNO055 |
| PD5, PD6 | USART2 TX, RX | Raspberry Pi 5 |
| PD8, PD9 | USART3 TX, RX | 디버그 출력 (ST-LINK 가상 COM 포트) |

> README의 배선도는 6월 기준이며, 이후 좌우 엔코더 배선을 서로 바꿨습니다. 실제 연결은 이 표를 기준으로 합니다.

## 빌드 방법

1. STM32CubeIDE에서 `firmware/AGV_Control.ioc`를 열고 코드 생성 (HAL 드라이버와 시작 코드가 생성됨)
2. `firmware/Core/Inc`, `firmware/Core/Src`의 파일을 생성된 프로젝트의 같은 폴더에 복사
3. 빌드할 로봇의 `firmware/robot1/main.c` 또는 `firmware/robot2/main.c`로 `Core/Src/main.c`를 교체한 뒤 빌드
4. BNO055 캘리브레이션 값이 없으면 `calibration/main.c`를 먼저 올려 플래시에 저장 (아래 절차)
5. 실행 중 디버그 로그는 USART3(ST-LINK 가상 COM 포트, 115200 bps)로 확인

## 비전 코드 실행

- 필요 패키지: `picamera2`, `opencv-contrib-python` (ArUco), `numpy`, `pyserial`
- 카메라 캘리브레이션 결과 파일 `camera_calibration.pkl`이 같은 폴더에 있어야 함
- 실행: `python3 vision/opencv_aruco_marker_detection.py` (라즈베리파이 `/dev/ttyAMA0`, 115200 bps로 STM32와 통신)

## BNO055 캘리브레이션 절차

1. `calibration/main.c`를 올리고 센서를 8자로 돌려 캘리브레이션 (NDOF 모드에서 모든 상태가 3이 될 때까지)
   - 캘리브레이션 상태가 모두 완료로 나오려면 지자기를 포함한 NDOF 모드가 필요해서 이 모드로 진행하고, 저장한 오프셋 중 가속도·자이로 값을 주행용 IMU 모드에서 그대로 사용
2. 완료되면 오프셋 22바이트를 플래시 Sector 11(`0x081C0000`)에 저장하고 다시 읽어 확인
3. 메인 펌웨어를 올리면 부팅할 때마다 이 값을 읽어 적용. 메인 펌웨어를 다시 올려도 Sector 11은 지워지지 않아 값이 유지됨
