/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : UART-command demo with BME68x sensor (I²C1) on STM32G071
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sx126x.h"
#include "sx126x_hal.h"
#include <bme68x.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>


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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 100
static char rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_char;
volatile uint8_t cmd_ready = 0; // Flag to indicate command is ready
volatile uint8_t cmd_buffer[RX_BUFFER_SIZE]; // Buffer for command processing
volatile uint8_t cmd_length = 0;             // Length of command
struct bme68x_dev bme;
int8_t bme_rslt;
/* LoRa Declarations */
const void *lora_context = NULL;
uint8_t emergency_stop = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                     void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                      void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);
void BME68x_Init(void);
void process_command(char *command);
void lora_tx_message(const char *message);
void lora_continuous_tx(void);
uint8_t lora_detect_hardware(void);
void lora_configure(void);
void debug_busy_pin(void);
void lora_wait_for_not_busy(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LoRa config ---------------------------------------------------------------*/
#define LORA_FREQ_HZ 868000000
#define LORA_TX_POWER 14
#define LORA_PAYLOAD_LEN 18
uint8_t lora_payload[LORA_PAYLOAD_LEN] = "Hello from STM32!";

// GPIO Pin definitions
#define LORA_RESET_PIN GPIO_PIN_5
#define LORA_RESET_PORT GPIOB
#define LORA_BUSY_PIN GPIO_PIN_11
#define LORA_BUSY_PORT GPIOB
#define LORA_NSS_PIN GPIO_PIN_10
#define LORA_NSS_PORT GPIOB
// Bosch driver-compatible functions
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                     void *intf_ptr) {
  I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
  uint8_t dev_addr = BME68X_I2C_ADDR_LOW << 1; // 0x76, change to _HIGH for 0x77
  if (HAL_I2C_Mem_Read(i2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len,
                       HAL_MAX_DELAY) == HAL_OK)
    return BME68X_OK;
  else
    return BME68X_E_COM_FAIL;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                      void *intf_ptr) {
  I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
  uint8_t dev_addr = BME68X_I2C_ADDR_LOW << 1; // 0x76, change to _HIGH for 0x77
  if (HAL_I2C_Mem_Write(i2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)data, len, HAL_MAX_DELAY) == HAL_OK)
    return BME68X_OK;
  else
    return BME68X_E_COM_FAIL;
}

void user_delay_us(uint32_t period, void *intf_ptr) {
  // Convert microseconds to milliseconds (round up)
  uint32_t ms = (period + 999) / 1000;
  HAL_Delay(ms);
}

void BME68x_Init(void) {
  bme.intf = BME68X_I2C_INTF;
  bme.read = user_i2c_read;
  bme.write = user_i2c_write;
  bme.delay_us = user_delay_us;
  bme.intf_ptr = &hi2c1;
  bme.amb_temp = 25;

  /* Step 1: Init device */
  bme_rslt = bme68x_init(&bme);
  if (bme_rslt != BME68X_OK) {
    // Debug: BME680 init failed
    const char *debug_msg = "[STM32] BME680 init failed!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg),
                      HAL_MAX_DELAY);
    Error_Handler();
  }

  // Debug: BME680 init successful
  const char *debug_msg = "[STM32] BME680 init successful!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg),
                    HAL_MAX_DELAY);

  /* Step 2: Configure oversampling and filter */
  struct bme68x_conf conf;
  bme_rslt = bme68x_get_conf(&conf, &bme);
  if (bme_rslt != BME68X_OK)
    Error_Handler();

  conf.os_hum = BME68X_OS_1X;      // Fastest humidity reading
  conf.os_temp = BME68X_OS_1X;     // Fastest temperature reading
  conf.os_pres = BME68X_OS_1X;     // Fastest pressure reading
  conf.filter = BME68X_FILTER_OFF; // No filter for fastest response

  bme_rslt = bme68x_set_conf(&conf, &bme);
  if (bme_rslt != BME68X_OK)
    Error_Handler();

  /* Step 3: Heater config (disabled for faster response) */
  struct bme68x_heatr_conf heatr_conf;
  heatr_conf.enable = BME68X_DISABLE; // Disable heater for faster readings
  heatr_conf.heatr_temp = 320;        // in °C
  heatr_conf.heatr_dur = 150;         // in ms

  bme_rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
  if (bme_rslt != BME68X_OK)
    Error_Handler();
}
void debug_busy_pin(void) {
  const char msg[] = "Debugging BUSY pin...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  for (int i = 0; i < 10; i++) {
    GPIO_PinState state = HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN);
    char debug_msg[50];
    sprintf(debug_msg, "BUSY pin %d: %s\r\n", i,
            (state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg),
                      HAL_MAX_DELAY);
    HAL_Delay(100);
  }
}

void lora_reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

void lora_wait_for_not_busy(void) {
  while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
    HAL_Delay(1);
  }
}

uint8_t lora_detect_hardware(void) {
  const char msg[] = "Detecting LoRa hardware (SPI)...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  sx126x_chip_status_t chip_status;
  sx126x_get_status(lora_context, &chip_status);

  // Print the raw SPI response for diagnosis
  char dbg[64];
  sprintf(dbg, "chip_mode: 0x%02X, cmd_status: 0x%02X\r\n",
          chip_status.chip_mode, chip_status.cmd_status);
  HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);

  if ((chip_status.chip_mode == 0x00 && chip_status.cmd_status == 0x00) ||
      (chip_status.chip_mode == 0xFF && chip_status.cmd_status == 0xFF)) {
    const char error[] = "ERROR: LoRa module not detected (SPI check)!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), HAL_MAX_DELAY);
    return 0;
  }

  const char success[] = "LoRa hardware detected (SPI check)!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)success, strlen(success),
                    HAL_MAX_DELAY);
  return 1;
}

void lora_configure(void) {
  const char msg[] = "Configuring LoRa module...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  const char dbg1[] = "Calling lora_detect_hardware...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)dbg1, strlen(dbg1), HAL_MAX_DELAY);

  if (!lora_detect_hardware()) {
    const char dbg2[] = "lora_detect_hardware returned 0\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg2, strlen(dbg2), HAL_MAX_DELAY);
    return;
  }

  const char dbg3[] = "lora_detect_hardware returned 1\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)dbg3, strlen(dbg3), HAL_MAX_DELAY);

  lora_reset();
  lora_wait_for_not_busy();

  sx126x_wakeup(lora_context);
  HAL_Delay(100);

  if (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
    const char error[] = "ERROR: LoRa module not responding after wakeup!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), HAL_MAX_DELAY);
    return;
  }

  sx126x_set_standby(lora_context, SX126X_STANDBY_CFG_RC);
  lora_wait_for_not_busy();

  sx126x_set_pkt_type(lora_context, SX126X_PKT_TYPE_LORA);
  lora_wait_for_not_busy();

  sx126x_set_rf_freq(lora_context, LORA_FREQ_HZ);
  lora_wait_for_not_busy();

  sx126x_mod_params_lora_t mod_params = {.sf = SX126X_LORA_SF7,
                                         .bw = SX126X_LORA_BW_125,
                                         .cr = SX126X_LORA_CR_4_5,
                                         .ldro = 0};
  sx126x_set_lora_mod_params(lora_context, &mod_params);
  lora_wait_for_not_busy();

  sx126x_pkt_params_lora_t pkt_params = {.preamble_len_in_symb = 8,
                                         .header_type =
                                             SX126X_LORA_PKT_EXPLICIT,
                                         .pld_len_in_bytes = LORA_PAYLOAD_LEN,
                                         .crc_is_on = true,
                                         .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(lora_context, &pkt_params);
  lora_wait_for_not_busy();

  sx126x_set_tx_params(lora_context, LORA_TX_POWER, SX126X_RAMP_200_US);
  lora_wait_for_not_busy();

  sx126x_set_buffer_base_address(lora_context, 0x00, 0x00);
  lora_wait_for_not_busy();

  sx126x_clear_irq_status(lora_context, SX126X_IRQ_ALL);
  lora_wait_for_not_busy();

  const char success[] = "LoRa module configured successfully!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)success, strlen(success),
                    HAL_MAX_DELAY);
}

void lora_tx_message(const char *message) {
  // Check hardware presence before TX
  if (!lora_detect_hardware()) {
    const char error[] = "ERROR: LoRa module not detected before TX!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), HAL_MAX_DELAY);
    return;
  }

  const char msg[] = "Sending LoRa message...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  debug_busy_pin();

  GPIO_PinState busy_state = HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN);
  char debug_msg[50];
  sprintf(debug_msg, "Current BUSY state: %s\r\n",
          (busy_state == GPIO_PIN_SET) ? "HIGH" : "LOW");
  HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg),
                    HAL_MAX_DELAY);

  if (busy_state == GPIO_PIN_SET) {
    const char busy_msg[] = "LoRa module is busy, waiting...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)busy_msg, strlen(busy_msg),
                      HAL_MAX_DELAY);
    lora_wait_for_not_busy();
  }

  sx126x_clear_irq_status(lora_context, SX126X_IRQ_ALL);
  lora_wait_for_not_busy();

  uint8_t msg_len = strlen(message);
  if (msg_len > LORA_PAYLOAD_LEN)
    msg_len = LORA_PAYLOAD_LEN;

  sx126x_write_buffer(lora_context, 0x00, (uint8_t *)message, msg_len);
  lora_wait_for_not_busy();

  sx126x_pkt_params_lora_t pkt_params = {.preamble_len_in_symb = 8,
                                         .header_type =
                                             SX126X_LORA_PKT_EXPLICIT,
                                         .pld_len_in_bytes = msg_len,
                                         .crc_is_on = true,
                                         .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(lora_context, &pkt_params);
  lora_wait_for_not_busy();

  sx126x_set_tx(lora_context, 5000);
  lora_wait_for_not_busy();

  const char waiting[] = "Waiting for transmission to complete...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)waiting, strlen(waiting),
                    HAL_MAX_DELAY);

  uint32_t timeout = 0;
  while (timeout < 5000) {
    sx126x_irq_mask_t irq_status;
    sx126x_get_irq_status(lora_context, &irq_status);

    if (irq_status & SX126X_IRQ_TX_DONE) {
      const char success[] = "LoRa TX completed successfully!\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)success, strlen(success),
                        HAL_MAX_DELAY);
      sx126x_clear_irq_status(lora_context, SX126X_IRQ_TX_DONE);

      sx126x_set_standby(lora_context, SX126X_STANDBY_CFG_RC);
      lora_wait_for_not_busy();
      return;
    }

    if (irq_status & SX126X_IRQ_TIMEOUT) {
      const char timeout_msg[] = "LoRa TX timeout!\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)timeout_msg, strlen(timeout_msg),
                        HAL_MAX_DELAY);
      sx126x_clear_irq_status(lora_context, SX126X_IRQ_TIMEOUT);

      sx126x_set_standby(lora_context, SX126X_STANDBY_CFG_RC);
      lora_wait_for_not_busy();
      return;
    }

    HAL_Delay(1);
    timeout++;
  }

  const char error[] = "LoRa TX operation timed out!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), HAL_MAX_DELAY);

  sx126x_set_standby(lora_context, SX126X_STANDBY_CFG_RC);
  lora_wait_for_not_busy();
}

void lora_continuous_tx(void) {
    const char msg[] = "Starting continuous LoRa transmission for scanner...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    const char help[] = "Look for signals around 868 MHz in your radio scanner\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)help, strlen(help), HAL_MAX_DELAY);
    const char stop_help[] = "Type 'stop' to stop transmission\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)stop_help, strlen(stop_help), HAL_MAX_DELAY);

    emergency_stop = 0;
    uint8_t counter = 0;

    while (!emergency_stop) {
        char message[50];
        sprintf(message, "STM32 LoRa Test Message #%d - Hello World!", counter++);

        sx126x_clear_irq_status(lora_context, SX126X_IRQ_ALL);
        lora_wait_for_not_busy();

        uint8_t msg_len = strlen(message);
        sx126x_write_buffer(lora_context, 0x00, (uint8_t*)message, msg_len);
        lora_wait_for_not_busy();

        sx126x_pkt_params_lora_t pkt_params = {
            .preamble_len_in_symb = 12,
            .header_type = SX126X_LORA_PKT_EXPLICIT,
            .pld_len_in_bytes = msg_len,
            .crc_is_on = true,
            .invert_iq_is_on = false
        };
        sx126x_set_lora_pkt_params(lora_context, &pkt_params);
        lora_wait_for_not_busy();

        sx126x_set_tx(lora_context, 1000);
        lora_wait_for_not_busy();

        uint32_t timeout = 0;
        while (timeout < 2000 && !emergency_stop) {
            sx126x_irq_mask_t irq_status;
            sx126x_get_irq_status(lora_context, &irq_status);

            if (irq_status & SX126X_IRQ_TX_DONE) {
                char success_msg[80];
                sprintf(success_msg, "Sent: %s\r\n", message);
                HAL_UART_Transmit(&huart2, (uint8_t *)success_msg, strlen(success_msg), HAL_MAX_DELAY);
                sx126x_clear_irq_status(lora_context, SX126X_IRQ_TX_DONE);
                break;
            }

            HAL_Delay(1);
            timeout++;
        }

        if (emergency_stop) {
            const char stop_msg[] = "Emergency stop requested!\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)stop_msg, strlen(stop_msg), HAL_MAX_DELAY);
            break;
        }

        sx126x_set_standby(lora_context, SX126X_STANDBY_CFG_RC);
        lora_wait_for_not_busy();

        HAL_Delay(3000);
    }

    const char end_msg[] = "Continuous transmission stopped.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)end_msg, strlen(end_msg), HAL_MAX_DELAY);
}

void process_command(char *command) {
  // Convert to lowercase for case-insensitive comparison
  for (int i = 0; command[i]; i++) {
    command[i] = tolower((unsigned char)command[i]);
  }

  if (strcmp(command, "start") == 0) {
    const char banner[] =
        "[STM32] Available commands:\r\n"
        "  temp      - Read temperature\r\n"
        "  pressure  - Read pressure\r\n"
        "  humidity  - Read humidity\r\n"
        "  add x y   - Add two numbers (e.g., add 5.5 3.2)\r\n"
        "  lora tx   - Send LoRa message (TBD)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)banner, strlen(banner),
                      HAL_MAX_DELAY);
  } else if (strcmp(command, "temp") == 0) {
    struct bme68x_data data;
    uint8_t n_fields;

    bme_rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (bme_rslt != BME68X_OK) {
      const char *msg = "[STM32] Failed to start measurement\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      return;
    }

    // Wait for measurement to complete
    struct bme68x_conf conf;
    bme68x_get_conf(&conf, &bme);
    uint32_t meas_dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme);

    // Debug: Show measurement duration
    char debug_buffer[64];
    snprintf(debug_buffer, sizeof(debug_buffer),
             "[DEBUG] Measurement duration: %lu us\r\n", meas_dur);
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buffer, strlen(debug_buffer),
                      HAL_MAX_DELAY);

    user_delay_us(meas_dur, &bme);

    bme_rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    if (bme_rslt == BME68X_OK && n_fields) {
      char temp_buffer[64];
      snprintf(temp_buffer, sizeof(temp_buffer),
               "[STM32] Temperature: %.2f °C\r\n", data.temperature);
      HAL_UART_Transmit(&huart2, (uint8_t *)temp_buffer, strlen(temp_buffer),
                        HAL_MAX_DELAY);
    } else {
      const char *msg = "[STM32] Sensor read failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  } else if (strcmp(command, "pressure") == 0) {
    struct bme68x_data data;
    uint8_t n_fields;

    bme_rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (bme_rslt != BME68X_OK) {
      const char *msg = "[STM32] Failed to start measurement\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      return;
    }

    // Wait for measurement to complete
    struct bme68x_conf conf;
    bme68x_get_conf(&conf, &bme);
    uint32_t meas_dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme);
    user_delay_us(meas_dur, &bme);

    bme_rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    if (bme_rslt == BME68X_OK && n_fields) {
      char pressure_buffer[64];
      snprintf(pressure_buffer, sizeof(pressure_buffer),
               "[STM32] Pressure: %.2f hPa\r\n", data.pressure / 100.0f);
      HAL_UART_Transmit(&huart2, (uint8_t *)pressure_buffer,
                        strlen(pressure_buffer), HAL_MAX_DELAY);
    } else {
      const char *msg = "[STM32] Sensor read failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  } else if (strcmp(command, "humidity") == 0) {
    struct bme68x_data data;
    uint8_t n_fields;

    bme_rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (bme_rslt != BME68X_OK) {
      const char *msg = "[STM32] Failed to start measurement\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      return;
    }

    // Wait for measurement to complete
    struct bme68x_conf conf;
    bme68x_get_conf(&conf, &bme);
    uint32_t meas_dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme);
    user_delay_us(meas_dur, &bme);

    bme_rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    if (bme_rslt == BME68X_OK && n_fields) {
      char humidity_buffer[64];
      snprintf(humidity_buffer, sizeof(humidity_buffer),
               "[STM32] Humidity: %.2f %%\r\n", data.humidity);
      HAL_UART_Transmit(&huart2, (uint8_t *)humidity_buffer,
                        strlen(humidity_buffer), HAL_MAX_DELAY);
    } else {
      const char *msg = "[STM32] Sensor read failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  } else if (strncmp(command, "add ", 4) == 0) {
    // Debug: Show the exact command received
    char debug_buffer[128];
    snprintf(debug_buffer, sizeof(debug_buffer),
             "[DEBUG] Command received: '%s' (length: %d)\r\n", command,
             strlen(command));
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buffer, strlen(debug_buffer),
                      HAL_MAX_DELAY);

    // Parse "add x y" command
    float x, y;
    int parsed = sscanf(command + 4, "%f %f", &x, &y);
    if (parsed == 2) {
      float result = x + y;
      char add_buffer[64];
      snprintf(add_buffer, sizeof(add_buffer), "[STM32] %g + %g = %g\r\n", x, y,
               result);
      HAL_UART_Transmit(&huart2, (uint8_t *)add_buffer, strlen(add_buffer),
                        HAL_MAX_DELAY);
    } else {
      const char *msg = "[STM32] Usage: add <number1> <number2>\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  } else if (strcmp(command, "lora tx") == 0) {
    lora_tx_message("Hello from STM32 LoRa!");
  } else if (strcmp(command, "lora scan") == 0) {
      lora_continuous_tx();
  } else {
    const char err[] = "[STM32] Unknown command\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_char == '\r') // Enter pressed
    {
      // Copy command to processing buffer
      for (int i = 0; i < rx_index; i++) {
        cmd_buffer[i] = rx_buffer[i];
      }
      cmd_buffer[rx_index] = '\0';
      cmd_length = rx_index;

      // Set flag to indicate command is ready
      cmd_ready = 1;

      // Reset buffer
      rx_index = 0;
    } else if (rx_index < RX_BUFFER_SIZE - 1) {
      rx_buffer[rx_index++] = rx_char;
    }

    // Restart reception
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize BME680 sensor
  BME68x_Init();

  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  const char *banner = "\r\n[STM32] UART+BME68x Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)banner, strlen(banner), HAL_MAX_DELAY);

  // Send initial prompt
  const char prompt[] = "> ";
  HAL_UART_Transmit(&huart2, (uint8_t *)prompt, strlen(prompt), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if command is ready to process
    if (cmd_ready) {
      // Convert volatile buffer to local buffer
      char local_cmd[RX_BUFFER_SIZE];
      for (int i = 0; i < cmd_length; i++) {
        local_cmd[i] = (char)cmd_buffer[i];
      }
      local_cmd[cmd_length] = '\0';

      // Process the command
      process_command(local_cmd);

      // Send newline and prompt
      const char newline[] = "\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)newline, strlen(newline),
                        HAL_MAX_DELAY);

      const char prompt[] = "> ";
      HAL_UART_Transmit(&huart2, (uint8_t *)prompt, strlen(prompt),
                        HAL_MAX_DELAY);

      // Clear the flag
      cmd_ready = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
