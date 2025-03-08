/* USER CODE BEGIN Header */
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "keypad.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "locked.h"
#include "unlocked.h"
#include "ring_buffer.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t keypad_colum_pressed = 0; // Variable para almacenar la columna presionada en el teclado
uint32_t key_pressed_tick = 0; // Marca de tiempo de la última tecla presionada
uint8_t b1_press_count = 0; // Contador de presiones del botón B1

uint8_t pc_rx_data[3]; // Buffer para datos recibidos desde la PC
ring_buffer_t pc_rx_buffer; // Declaración del buffer circular para la PC

uint8_t keypad_rx_data[3]; // Buffer para datos recibidos desde el teclado
ring_buffer_t keypad_rx_buffer; // Declaración del buffer circular para el teclado

uint8_t internet_rx_data[3]; // Buffer para datos recibidos desde internet
ring_buffer_t internet_rx_buffer; // Declaración del buffer circular para internet

extern char state[3]; // Declaración externa de la variable de estado
char state[3] = "Close"; // Inicialización del estado de la puerta como "Cerrada"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int system_events_handler(char *event)
{
  
}
void system_state_machine(char *states)
{

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == B1_Pin) { // Si la interrupción proviene del pin del botón B1
      int press_type = detect_button_press(); // Detecta el tipo de presión del botón
      if (press_type == 2) { // Si es una presión doble
        strcpy(state, "Open"); // Cambia el estado a "Abierto"
        HAL_UART_Transmit(&huart2, (uint8_t *)"Temporaly open\r\n", 15, 100); // Envía mensaje por UART
        HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_SET); // Activa el pin de la puerta (abre)
        ssd1306_Fill(Black); // Limpia la pantalla OLED
        ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White); // Muestra la imagen de "abierto"
        ssd1306_UpdateScreen(); // Actualiza la pantalla OLED
        HAL_Delay(5000);  // Mantiene la puerta abierta por 5 segundos
        strcpy(state, "Close"); // Cambia el estado a "Cerrado"
        HAL_UART_Transmit(&huart2, (uint8_t *)"close door\r\n", 15, 100); // Envía mensaje por UART
        HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET); // Desactiva el pin de la puerta (cierra)
        ssd1306_Fill(Black); // Limpia la pantalla OLED
        ssd1306_DrawBitmap(0, 0, locked, 128, 64, White); // Muestra la imagen de "cerrado"
        ssd1306_UpdateScreen(); // Actualiza la pantalla OLED
      }
      else if (press_type == 1) { // Si es una presión 
        if (strcmp(state, "Open") == 0){  // Si la puerta está abierta
          strcpy(state, "Close"); // Cambia el estado a "Cerrado"
          HAL_UART_Transmit(&huart2, (uint8_t *)"close door\r\n", 15, 100); // Envía mensaje por UART
          HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET); // Cierra la puerta
          ssd1306_Fill(Black); // Limpia la pantalla OLED
          ssd1306_DrawBitmap(0, 0, locked, 128, 64, White); // Muestra la imagen de "cerrado"
          ssd1306_UpdateScreen(); // Actualiza la pantalla OLED
        }
        else {  // Si la puerta está cerrada
          strcpy(state, "Open");  // Cambia el estado a "Abierto"
          HAL_UART_Transmit(&huart2, (uint8_t *)"opened door\r\n", 15, 100); // Envía mensaje por UART
          HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_SET); // Abre la puerta
          ssd1306_Fill(Black); // Limpia la pantalla OLED
          ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White); // Muestra la imagen de "abierto"
          ssd1306_UpdateScreen(); // Actualiza la pantalla OLED
        }
      }
  } else { 
      keypad_colum_pressed = GPIO_Pin; // Si la interrupción proviene del teclado, guarda la columna presionada
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) { //Si la recepción proviene de UART2(PC)
    HAL_UART_Transmit(&huart3, huart->pRxBuffPtr, huart->RxXferSize, 1000);
    HAL_UART_Receive_IT(&huart2, huart->pRxBuffPtr, huart->RxXferSize);
  } else if (huart->Instance == USART3) { //Si la recepción proviene de UART3 (ESP01)
    HAL_UART_Transmit(&huart2, huart->pRxBuffPtr, huart->RxXferSize, 1000);
    HAL_UART_Receive_IT(&huart3, huart->pRxBuffPtr, huart->RxXferSize);
  }
}

/**
 * @brief  Heartbeat function to blink LED2 every 1 second to indicate the system is running
*/
void heartbeat(void)
{
  static uint32_t last_heartbeat = 0;
  if (HAL_GetTick() - last_heartbeat > 1000) // Verifica si ha pasado 1 segundo
  {
    last_heartbeat = HAL_GetTick();
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Alterna el estado del LED
  }
}

/**
 * @brief  Retargets the C library printf function to the USART.
*/
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}

void ssd_printer(ring_buffer_t *rb, uint8_t *buffer, char *state)
{
  (void)rb;
  if (strcmp((char *)buffer, "#A#") == 0) //Comando para abrir la puerta
  {
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
    ssd1306_UpdateScreen();
  }

 else if (strcmp((char *)buffer, "#C#") == 0) //Comando para cerrar la puerta
 {
  
  ssd1306_Fill(Black);
  ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
  ssd1306_UpdateScreen();
  
 }
}

void process_command(ring_buffer_t *rb, uint8_t *buffer, char *state) { //Procesa los comandos de los ring buffers
  if (ring_buffer_size(rb) == 3) {   //Comprueba si el ring buffer está lleno
      for (int i = 0; i < 3; i++) {
          ring_buffer_read(rb, &buffer[i]);
      }
      buffer[3] = '\0';  

      if (strcmp((char *)buffer, "#A#") == 0) { //Verifica si es el comando para abrir
          if (strcmp(state, "Open") == 0) { //Verifica si la puerta está abierta
              HAL_UART_Transmit(&huart2, (uint8_t *)"Already opened\r\n", 18, 100);
              ssd_printer(rb, buffer, state);
          } else {
              HAL_GPIO_TogglePin(DOOR_GPIO_Port, DOOR_Pin);  //Si está cerrada la abre
              HAL_UART_Transmit(&huart2, (uint8_t *)"opened door\r\n", 15, 100);
              ssd_printer(rb, buffer, state);
              strcpy(state, "Open");
          }

      } 

      else if (strcmp((char *)buffer, "#C#") == 0) {//Verifica si es el comando de cerrar
          if (strcmp(state, "Close") == 0) { //Verifica si la puerta está cerrada
              HAL_UART_Transmit(&huart2, (uint8_t *)"Already closed\r\n", 17, 100);
              ssd_printer(rb, buffer, state);
          } else {
              HAL_GPIO_TogglePin(DOOR_GPIO_Port, DOOR_Pin); //Si está abierta la abre 
              HAL_UART_Transmit(&huart2, (uint8_t *)"closed door\r\n", 15, 100);
              ssd_printer(rb, buffer, state);
              strcpy(state, "Close");
          }
      }  
      else {
          HAL_UART_Transmit(&huart2, (uint8_t *)"invalid command\r\n", 18, 100); //Si no es un comando válido
          if (strcmp(state, "Close") == 0) { //Si la puerta está cerrada la deja en ese estado
              ssd1306_Fill(Black);
              ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
              ssd1306_UpdateScreen();
          } else { //Si la puerta está abierta la deja en ese estado
              ssd1306_Fill(Black);
              ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
              ssd1306_UpdateScreen();
          }
      }

      for (int i = 0; i < 3; i++) {
          buffer[i] = '_';
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart2, pc_rx_data, sizeof(pc_rx_data));
  //HAL_UART_Receive_IT(&huart3, keypad_rx_data, sizeof(keypad_rx_data));
  
  ring_buffer_init(&pc_rx_buffer, pc_rx_data, sizeof(pc_rx_data)); //Inicialización de los ring buffer
  ring_buffer_init(&keypad_rx_buffer, keypad_rx_data, sizeof(keypad_rx_data));//Inicialización de los ring buffer
  ring_buffer_init(&internet_rx_buffer, internet_rx_data, sizeof(internet_rx_data));//Inicialización de los ring buffer
  setvbuf(stdout, NULL, _IONBF, 0);  

  keypad_init(); //Inicialización del keypad
  HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET);
  ssd1306_Init(); //Inicialización de la pantalla
  ssd1306_Fill(Black);
  ssd1306_DrawBitmap(0, 0, locked, 128, 64, White); //Estado inicial de la puerta (cerrada)
  ssd1306_UpdateScreen();
  uint8_t internet_key;
  uint8_t pc_key;
  uint8_t key;

  for (size_t i = 0; i < sizeof(keypad_rx_data); i++) {
    keypad_rx_data[i] = '_';
    pc_rx_data[i] = '_';
  } 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    heartbeat(); 
    if (keypad_colum_pressed != 0 && (key_pressed_tick + 5) < HAL_GetTick()) {
      key = keypad_scan(keypad_colum_pressed);
      keypad_colum_pressed = 0;
      if (key != 'E') {
        ring_buffer_write(&keypad_rx_buffer, key);
        uint8_t size = ring_buffer_size(&keypad_rx_buffer);
        char msg[45];
        snprintf(msg, sizeof(msg), "Key: %c, Buffer: %s, Size: %d\r\n", key, keypad_rx_data, size);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
      }
      
    }
    if (HAL_UART_Receive(&huart2, &pc_key, 1, 10) == HAL_OK) {
      ring_buffer_write(&pc_rx_buffer, pc_key);
      uint8_t size = ring_buffer_size(&pc_rx_buffer);
      char msg[45];
      snprintf(msg, sizeof(msg), "PC Key: %c, Buffer: %s, Size: %d\r\n", pc_key, pc_rx_data, size);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    }
    if (HAL_UART_Receive(&huart3, &internet_key, 1, 10) == HAL_OK) {
      ring_buffer_write(&internet_rx_buffer, internet_key);
      uint8_t size = ring_buffer_size(&internet_rx_buffer);
      char msg[45];
      snprintf(msg, sizeof(msg), "PC Key: %c, Buffer: %s, Size: %d\r\n", internet_key, internet_rx_data, size);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    }
    process_command(&keypad_rx_buffer, keypad_rx_data, state); //Procesamiento del ring buffer de los datos ingreados
    process_command(&pc_rx_buffer, pc_rx_data, state);//Procesamiento del ring buffer de los datos
    process_command(&internet_rx_buffer, internet_rx_data, state);//Procesamiento del ring buffer de los datos
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    system_state_machine(state);
    HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  hi2c1.Init.Timing = 0x10D19CE4;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DOOR_Pin|LD2_Pin|LD1_Pin|ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RING_Pin */
  GPIO_InitStruct.Pin = RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOOR_Pin LD2_Pin LD1_Pin */
  GPIO_InitStruct.Pin = DOOR_Pin|LD2_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_4_Pin */
  GPIO_InitStruct.Pin = COLUMN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_2_Pin COLUMN_3_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin|COLUMN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
