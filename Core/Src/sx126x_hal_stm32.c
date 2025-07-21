/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for NUCLEO-G071RB LoRa UART menu
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "sx126x_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

// GPIO Pin definitions (matching main.c)
#define LORA_RESET_PIN GPIO_PIN_5
#define LORA_RESET_PORT GPIOB
#define LORA_BUSY_PIN GPIO_PIN_11
#define LORA_BUSY_PORT GPIOB
#define LORA_NSS_PIN GPIO_PIN_12
#define LORA_NSS_PORT GPIOB

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    // Reset sequence for LoRa module
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    
    // Wait for BUSY pin to go low (not busy)
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    // Wait for BUSY pin to go low
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);  // NSS low
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);    // NSS high
    HAL_Delay(1);
    
    // Wait for BUSY pin to go low again
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    // Wait for BUSY pin to go low
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);  // NSS low
    
    // Send command
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t*)command, command_length, 1000);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
        return SX126X_HAL_STATUS_ERROR;
    }
    
    // Send data if provided
    if (data != NULL && data_length > 0) {
        status = HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_length, 1000);
    }
    
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);    // NSS high
    
    return (status == HAL_OK) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    // Wait for BUSY pin to go low
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);  // NSS low
    
    // Send command
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t*)command, command_length, 1000);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
        return SX126X_HAL_STATUS_ERROR;
    }

    // Read data
    status = HAL_SPI_Receive(&hspi1, data, data_length, 1000);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);    // NSS high

    return (status == HAL_OK) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_verify_communication(void) {
    // Wait for BUSY pin to go low
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }
    
    uint8_t tx_buf[2] = {0xC0, 0x00}; // Command + dummy
    uint8_t rx_buf[2] = {0};

    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 100);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    // rx_buf[1] now contains device status
    if (status == HAL_OK && rx_buf[1] != 0x00 && rx_buf[1] != 0xFF) {
        return SX126X_HAL_STATUS_OK;
    }
    return SX126X_HAL_STATUS_ERROR;
}
