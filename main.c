/* USER CODE BEGIN Header */
/* version 2*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h> // Para funciones matemáticas como asin()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definiciones para el MPU6050
#define MPU6050_ADDR          0x68 // Dirección base del MPU6050
#define MPU6050_REG_PWR_MGMT  0x6B
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG     0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_GYRO_XOUT_H 0x43

// Configuración de servo (valores típicos para SG90)
// Modificados para un timer de 50Hz (20ms) - Valores típicos para el SG90
#define SERVO_MIN_PULSE 220  // ~1ms (posición 0°)
#define SERVO_MAX_PULSE 1040 // ~2ms (posición 180°)

// Factor para filtro complementario
#define ALPHA 0.99f // Aumentado para reducir drift del acelerómetro
#define MOVEMENT_THRESHOLD 0.3f // Reducido para detectar solo movimientos reales
#define MAX_ANGLE_CHANGE 5.0f // Mantener para movimiento completo
#define FILTER_SIZE 5 // Mantener el valor actual
#define DEADZONE 0.5f // Reducido para mejor respuesta
#define KALMAN_Q 0.0001f // Reducido para menos corrección
#define KALMAN_R 0.1f // Mantener el valor actual
#define ORIGIN_THRESHOLD 2.0f // Reducido para que solo retorne cuando esté muy cerca
#define STABLE_TIME 15000 // Aumentado a 15 segundos para requerir más tiempo de estabilidad
#define RETURN_SPEED 0.1f // Reducido para retorno más suave
#define HORIZONTAL_THRESHOLD 5.0f // Umbral para considerar posición horizontal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Variables para filtrado y calibración
int16_t accel_offset_x = 0;
int16_t accel_offset_y = 0;
int16_t accel_offset_z = 0;
int16_t gyro_offset_x = 0;
float filtered_angle = 90.0f;  // Empezamos en posición central
float last_filtered_angle = 90.0f; // Para el filtro de movimiento
float angle_history[FILTER_SIZE]; // Historial para el filtro de promedio móvil
uint8_t history_index = 0; // Índice actual para el historial
uint8_t i2c_status = 0; // Para depuración I2C

// Variables para el filtro de Kalman
float kalman_angle = 90.0f;
float kalman_bias = 0.0f;
float kalman_P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

// Variables para control de estabilidad
float target_angle = 90.0f;
uint32_t last_stable_time = 0;
float last_stable_angle = 90.0f;
uint8_t is_near_origin = 0; // Flag para posición cerca del origen
uint8_t should_return_to_zero = 0; // Flag para retornar a cero
uint8_t is_horizontal = 0; // Flag para posición horizontal
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
int16_t Read_Accel_X(void);
int16_t Read_Accel_Y(void);
int16_t Read_Accel_Z(void);
int16_t Read_Gyro_X(void);
float Get_Accel_Angle(void);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);
void Calibrate_Sensors(void);
void Test_Servo(void); // Función de prueba para el servo
uint8_t MPU6050_IsReady(void); // Función para verificar comunicación con MPU6050
void Kalman_Update(float new_angle, float new_rate, float dt);
float NonLinear_Map(float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Verifica si el MPU6050 está respondiendo
uint8_t MPU6050_IsReady(void) {
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c3, MPU6050_ADDR<<1, 1, 100);
    if (status == HAL_OK) {
        return 1;
    }
    return 0;
}

// Función para configurar el MPU6050
void MPU6050_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;

    // Verificar comunicación con el MPU6050
    if (!MPU6050_IsReady()) {
        // Error de comunicación - Parpadear LED rápidamente para indicar error
        for (int i = 0; i < 10; i++) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        }
        i2c_status = 1; // Marcar error
        return; // No continuar si no hay comunicación
    }

    // Despertar el MPU6050
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_PWR_MGMT, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        i2c_status = 2; // Marcar error de escritura
        return;
    }

    HAL_Delay(50); // Dar tiempo para despertar

    // Configurar tasa de muestreo a 1kHz
    data = 0x07;
    status = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        i2c_status = 3;
        return;
    }

    // Configurar filtro paso bajo a 5Hz
    data = 0x06;
    status = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        i2c_status = 4;
        return;
    }

    // Configurar giroscopio a ±250°/s
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        i2c_status = 5;
        return;
    }

    // Configurar acelerómetro a ±2g
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        i2c_status = 6;
        return;
    }

    HAL_Delay(100); // Esperar a que se estabilice
}

// Función para probar el servo moviendo a posiciones extremas
void Test_Servo(void) {
    // Probar todo el rango de movimiento en incrementos
    for (int angle = 0; angle <= 180; angle += 45) {
        Set_Servo_Angle(&htim2, TIM_CHANNEL_1, angle);
        HAL_Delay(500);  // Medio segundo en cada posición
    }

    // Volver al centro
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 90);
    HAL_Delay(1000);

    // Prueba específica de ángulos extremos
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 0);
    HAL_Delay(1000);
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 180);
    HAL_Delay(1000);
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 90);
    HAL_Delay(1000);
}

// Función para controlar el servo
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle) {
    // Asegurar que el ángulo está en el rango 0-180
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Mapear el ángulo al rango de PWM
    uint32_t pulse = SERVO_MIN_PULSE + ((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);

    // Establecer el valor del comparador para el PWM
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

// Función para leer el acelerómetro X
int16_t Read_Accel_X(void) {
    uint8_t accelData[2];
    HAL_StatusTypeDef status;

    // Usar timeout limitado (100ms)
    status = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_XOUT_H, 1, accelData, 2, 100);

    if (status != HAL_OK) {
        // Parpadear LED para indicar error de lectura
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        return 0; // En caso de error, devolver 0
    }

    return (int16_t)(accelData[0] << 8 | accelData[1]);
}

// Función para leer el acelerómetro Y
int16_t Read_Accel_Y(void) {
    uint8_t accelData[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_YOUT_H, 1, accelData, 2, 100);

    if (status != HAL_OK) {
        return 0; // En caso de error, devolver 0
    }

    return (int16_t)(accelData[0] << 8 | accelData[1]);
}

// Función para leer el acelerómetro Z
int16_t Read_Accel_Z(void) {
    uint8_t accelData[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_ZOUT_H, 1, accelData, 2, 100);

    if (status != HAL_OK) {
        return 0; // En caso de error, devolver 0
    }

    return (int16_t)(accelData[0] << 8 | accelData[1]);
}

// Función para leer el giroscopio X
int16_t Read_Gyro_X(void) {
    uint8_t gyroData[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR<<1, MPU6050_REG_GYRO_XOUT_H, 1, gyroData, 2, 100);
    if (status != HAL_OK) {
        return 0; // En caso de error, devolver 0
    }

    return (int16_t)(gyroData[0] << 8 | gyroData[1]);
}

// Función para calcular el ángulo desde el acelerómetro
float Get_Accel_Angle(void) {
    // Para estabilización lateral, usamos Y y Z
    int16_t accelY = Read_Accel_Y() - accel_offset_y;
    int16_t accelZ = Read_Accel_Z() - accel_offset_z;

    // Calcular el ángulo usando arctangente
    float angle = atan2f((float)accelY, (float)accelZ) * 57.296f; // 57.296 = 180/PI

    // Ajustar el ángulo para que esté en el rango 0-180 para el servo
    angle = angle + 90.0f;

    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    return angle;
}

// Función para calibrar los sensores
void Calibrate_Sensors(void) {
    int32_t accel_x_sum = 0;
    int32_t accel_y_sum = 0;
    int32_t accel_z_sum = 0;
    int32_t gyro_x_sum = 0;
    const int samples = 50; // Número de muestras para promediar

    // Tomar múltiples muestras para promediar
    for (int i = 0; i < samples; i++) {
        accel_x_sum += Read_Accel_X();
        accel_y_sum += Read_Accel_Y();
        accel_z_sum += Read_Accel_Z();
        gyro_x_sum += Read_Gyro_X();
        HAL_Delay(10);
    }

    // Calcular offsets
    accel_offset_x = accel_x_sum / samples;
    accel_offset_y = accel_y_sum / samples;
    accel_offset_z = accel_z_sum / samples;
    gyro_offset_x = gyro_x_sum / samples;
}

// Función para aplicar el filtro de Kalman
void Kalman_Update(float new_angle, float new_rate, float dt) {
    // Predicción
    kalman_angle += dt * (new_rate - kalman_bias);
    kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + KALMAN_Q);
    kalman_P[0][1] -= dt * kalman_P[1][1];
    kalman_P[1][0] -= dt * kalman_P[1][1];
    kalman_P[1][1] += KALMAN_Q * dt;

    // Actualización
    float S = kalman_P[0][0] + KALMAN_R;
    float K[2] = {kalman_P[0][0] / S, kalman_P[1][0] / S};

    float y = new_angle - kalman_angle;
    kalman_angle += K[0] * y;
    kalman_bias += K[1] * y;

    float P00_temp = kalman_P[0][0];
    float P01_temp = kalman_P[0][1];

    kalman_P[0][0] -= K[0] * P00_temp;
    kalman_P[0][1] -= K[0] * P01_temp;
    kalman_P[1][0] -= K[1] * P00_temp;
    kalman_P[1][1] -= K[1] * P01_temp;
}

// Función para mapeo no lineal del ángulo
float NonLinear_Map(float angle) {
    float center = target_angle;
    float diff = angle - center;

    // Aplicar zona muerta
    if (fabsf(diff) < DEADZONE) {
        return center;
    }

    // Mapeo no lineal más suave
    float sign = (diff > 0) ? 1.0f : -1.0f;
    float abs_diff = fabsf(diff) - DEADZONE;
    float mapped_diff = sign * (abs_diff * 0.8f); // Reducido para movimientos más suaves

    return center + mapped_diff;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t last_time = 0;
  float dt = 0;
  float gyro_angle = 90.0f;  // Empezamos en posición central
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
  MX_TIM2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  // Iniciar el PWM para el servo
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Probar el servo primero para verificar su funcionamiento
  Test_Servo();

  // Inicializar MPU6050 con configuración completa
  MPU6050_Init();

  // Si hay problemas con I2C, mantener el servo en posición central y mostrar error
  if (i2c_status != 0) {
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 90);
    while (1) {
      // Parpadear con un patrón específico según el tipo de error
      for (int i = 0; i < i2c_status; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(100);
      }
      HAL_Delay(1000); // Pausa entre secuencias
    }
  }

  // Calibrar sensores (mantener el dispositivo estable y horizontal)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED encendido durante calibración
  Calibrate_Sensors();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED apagado después de calibración

  // Inicializar tiempo
  last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Calcular tiempo transcurrido
    uint32_t current_time = HAL_GetTick();
    dt = (current_time - last_time) / 1000.0f;  // Convertir a segundos
    last_time = current_time;

    // Asegurar que dt es razonable (evitar divisiones por cero o valores muy pequeños)
    if (dt < 0.001f) dt = 0.001f;

    // Leer sensores
    int16_t gyro_x = Read_Gyro_X();

    // Calcular ángulo desde acelerómetro
    float accel_angle = Get_Accel_Angle();

    // Calcular ángulo desde giroscopio (integración)
    // Factor 131.0 es para convertir a grados/s con rango ±250°/s
    float gyro_rate = (float)(gyro_x - gyro_offset_x) / 131.0f;
    gyro_angle += gyro_rate * dt;

    // Limitar gyro_angle al rango 0-180 para el servo
    if (gyro_angle < 0.0f) gyro_angle = 0.0f;
    if (gyro_angle > 180.0f) gyro_angle = 180.0f;

    // Filtro complementario para combinar mediciones
    filtered_angle = ALPHA * (filtered_angle + gyro_rate * dt) + (1.0f - ALPHA) * accel_angle;

    // Aplicar filtro de Kalman
    Kalman_Update(filtered_angle, gyro_rate, dt);
    filtered_angle = kalman_angle;

    // Detectar si está cerca del origen (90 grados)
    float angle_to_origin = fabsf(filtered_angle - 90.0f);
    is_near_origin = (angle_to_origin < ORIGIN_THRESHOLD);

    // Calcular el cambio de ángulo
    float angle_change = fabsf(filtered_angle - last_stable_angle);
    uint8_t is_moving = (angle_change > MOVEMENT_THRESHOLD);

    // Solo considerar estabilidad si está muy cerca del origen
    if (is_near_origin && !is_moving) {
        if (current_time - last_stable_time > STABLE_TIME) {
            should_return_to_zero = 1;
        }
    } else {
        // Si se mueve o está lejos del origen, resetear todo
        last_stable_time = current_time;
        last_stable_angle = filtered_angle;
        should_return_to_zero = 0;
    }

    // Si debe retornar a cero, mover muy gradualmente
    if (should_return_to_zero) {
        if (filtered_angle > 90.0f) {
            filtered_angle -= RETURN_SPEED;
            if (filtered_angle < 90.0f) filtered_angle = 90.0f;
        } else if (filtered_angle < 90.0f) {
            filtered_angle += RETURN_SPEED;
            if (filtered_angle > 90.0f) filtered_angle = 90.0f;
        }
    }

    // Actualizar el último ángulo estable
    last_stable_angle = filtered_angle;

    // Limitar el valor del ángulo para el servo
    if (filtered_angle < 0.0f) filtered_angle = 0.0f;
    if (filtered_angle > 180.0f) filtered_angle = 180.0f;

    // Aplicar filtro de promedio móvil
    angle_history[history_index] = filtered_angle;
    history_index = (history_index + 1) % FILTER_SIZE;

    float sum = 0.0f;
    for(int i = 0; i < FILTER_SIZE; i++) {
        sum += angle_history[i];
    }
    filtered_angle = sum / FILTER_SIZE;

    // Aplicar mapeo no lineal
    filtered_angle = NonLinear_Map(filtered_angle);

    // Limitar la velocidad de cambio
    angle_change = fabsf(filtered_angle - last_filtered_angle);
    if (angle_change > MAX_ANGLE_CHANGE) {
        if (filtered_angle > last_filtered_angle) {
            filtered_angle = last_filtered_angle + MAX_ANGLE_CHANGE;
        } else {
            filtered_angle = last_filtered_angle - MAX_ANGLE_CHANGE;
        }
    }

    last_filtered_angle = filtered_angle;

    // Establecer el ángulo del servo (invertido para compensar el movimiento)
    Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 180 - (uint8_t)filtered_angle);

    // Parpadear LED para indicar funcionamiento
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // Delay para una tasa de actualización estable (50Hz)
    HAL_Delay(20);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

