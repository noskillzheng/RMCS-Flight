/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : WildFire STM32F103C8T6 RGB LED Controller (USART1 + SWD Mailbox)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SWD_MAILBOX_MAGIC 0x4C454431UL /* "LED1" */

typedef enum
{
  LED_EFFECT_STATIC = 0U,
  LED_EFFECT_FLOW   = 1U,
  LED_EFFECT_BLINK  = 2U,
  LED_EFFECT_BREATH = 3U
} LedEffectMode;

/**
 * @brief Race-free 4-word memory-mapped SWD mailbox at 0x20000000 (.swd_mailbox):
 *   0x20000000 (magic)  : 0x4C454431 ("LED1") once initialized
 *   0x20000004 (cmd)    : Host writes ASCII command in [7:0]; MCU clears to 0 after handling
 *   0x20000008 (status) : Written only by MCU:
 *                           bits [7:0]   = current physical LED mask (bit0=R/PA1, bit1=G/PA2, bit2=B/PA3)
 *                           bits [15:8]  = LedEffectMode (0=STATIC, 1=FLOW, 2=BLINK, 3=BREATH)
 *                           bits [23:16] = last accepted ASCII command
 *                           bits [31:24] = base/target color mask
 *   0x2000000C (seq)    : Incremented by MCU after each accepted command
 */
typedef struct
{
  volatile uint32_t magic;
  volatile uint32_t cmd;
  volatile uint32_t status;
  volatile uint32_t seq;
} SwdLedMailbox;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLOW_INTERVAL_MS   280U
#define BLINK_INTERVAL_MS  250U
#define BREATH_PERIOD_MS   1600U
#define BREATH_PWM_STEPS   256U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Placed at 0x20000000 (.swd_mailbox section) for zero-halt SWD terminal control */
__attribute__((section(".swd_mailbox"), used)) static SwdLedMailbox g_swd_mailbox;

static LedEffectMode g_effect_mode = LED_EFFECT_STATIC;
static uint8_t g_static_mask = 0U;
static uint8_t g_effect_base_mask = 0x01U;
static uint8_t g_current_mask = 0U;
static uint8_t g_last_cmd = '0';
static uint8_t g_blink_on = 1U;
static uint8_t g_flow_step = 0U;
static uint32_t g_effect_last_tick = 0U;
static uint32_t g_breath_pwm_ticks = 72000U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t str_length(const char *str);
static void uart_send_str(const char *str);
static void uart_send_crlf(void);
static const char *mask_color_name(uint8_t mask);
static void sync_mailbox_status(void);
static void write_led_pins(uint8_t mask);
static void apply_led_mask(uint8_t mask);
static void send_uart_banner(void);
static void send_uart_ack(uint8_t cmd, const char *mode_desc, uint8_t mask);
static uint8_t handle_command_byte(uint8_t cmd, uint8_t send_ack);
static void update_dynamic_effect(void);
static void init_cycle_counter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t str_length(const char *str)
{
  uint16_t len = 0U;
  while ((uint8_t)str[len] != 0x00U)
  {
    len++;
  }
  return len;
}

static void uart_send_str(const char *str)
{
  USART1_WriteBytes((const uint8_t *)str, str_length(str));
}

static void uart_send_crlf(void)
{
  static const uint8_t kCrlf[2] = {0x0DU, 0x0AU};
  USART1_WriteBytes(kCrlf, sizeof(kCrlf));
}

static const char *mask_color_name(uint8_t mask)
{
  switch (mask & 0x07U)
  {
    case 0x00U: return "OFF";
    case 0x01U: return "RED";
    case 0x02U: return "GREEN";
    case 0x03U: return "YELLOW (R+G)";
    case 0x04U: return "BLUE";
    case 0x05U: return "PURPLE (R+B)";
    case 0x06U: return "CYAN (G+B)";
    case 0x07U: return "WHITE (R+G+B)";
    default:    return "OFF";
  }
}

static void sync_mailbox_status(void)
{
  uint8_t target_mask = (g_effect_mode == LED_EFFECT_STATIC)
                            ? (g_static_mask & 0x07U)
                            : (g_effect_base_mask & 0x07U);
  g_swd_mailbox.status = ((uint32_t)(g_current_mask & 0x07U))
                       | ((uint32_t)g_effect_mode << 8U)
                       | ((uint32_t)g_last_cmd << 16U)
                       | ((uint32_t)target_mask << 24U);
}

static void write_led_pins(uint8_t mask)
{
  uint8_t clean_mask = mask & 0x07U;
  /* Active-low RGB LEDs on WildFire STM32F103C8T6: PA1=Red, PA2=Green, PA3=Blue */
  uint32_t set_pins = 0U;
  uint32_t reset_pins = 0U;

  if ((clean_mask & 0x01U) != 0U) { reset_pins |= LED_R_Pin; } else { set_pins |= LED_R_Pin; }
  if ((clean_mask & 0x02U) != 0U) { reset_pins |= LED_G_Pin; } else { set_pins |= LED_G_Pin; }
  if ((clean_mask & 0x04U) != 0U) { reset_pins |= LED_B_Pin; } else { set_pins |= LED_B_Pin; }

  GPIOA->BSRR = set_pins | (reset_pins << 16U);
  g_current_mask = clean_mask;
}

static void apply_led_mask(uint8_t mask)
{
  write_led_pins(mask);
  sync_mailbox_status();
}

static void send_uart_banner(void)
{
  uart_send_crlf();
  uart_send_str(
      "[Fire_F103 RGB Ready] Commands: r=Red g=Green b=Blue | "
      "y=Yellow c=Cyan m=Purple w=White | f=Flow k=Blink e=Breath o/0=Off ?=Help");
  uart_send_crlf();
}

static void send_uart_ack(uint8_t cmd, const char *mode_desc, uint8_t mask)
{
  uint8_t prefix[6] = {'O', 'K', ':', cmd, '=', (uint8_t)('0' + (mask & 0x07U))};
  USART1_WriteBytes(prefix, sizeof(prefix));
  uart_send_str(" [");
  uart_send_str(mode_desc);
  uart_send_str("]");
  uart_send_crlf();
}

static uint8_t handle_command_byte(uint8_t cmd, uint8_t send_ack)
{
  /* Ignore whitespace and line endings so both raw and line-buffered terminals work */
  if (cmd == 0x0DU || cmd == 0x0AU || cmd == ' ' || cmd == 0x09U)
  {
    return 0U;
  }

  if (cmd == '?' || cmd == 'h' || cmd == 'H')
  {
    if (send_ack != 0U)
    {
      send_uart_banner();
    }
    return 1U;
  }

  if (cmd == 'f' || cmd == 'F')
  {
    g_effect_mode = LED_EFFECT_FLOW;
    g_flow_step = 0U;
    g_last_cmd = cmd;
    g_effect_base_mask = 0x01U;
    g_effect_last_tick = HAL_GetTick();
    apply_led_mask(0x01U);
    g_swd_mailbox.seq++;
    if (send_ack != 0U)
    {
      send_uart_ack(cmd, "FLOW (R->Y->G->C->B->M)", 0x01U);
    }
    return 1U;
  }

  if (cmd == 'k' || cmd == 'K')
  {
    g_effect_mode = LED_EFFECT_BLINK;
    if ((g_static_mask & 0x07U) != 0U)
    {
      g_effect_base_mask = g_static_mask & 0x07U;
    }
    else if ((g_effect_base_mask & 0x07U) == 0U)
    {
      g_effect_base_mask = 0x07U;
    }
    g_blink_on = 1U;
    g_last_cmd = cmd;
    g_effect_last_tick = HAL_GetTick();
    apply_led_mask(g_effect_base_mask);
    g_swd_mailbox.seq++;
    if (send_ack != 0U)
    {
      send_uart_ack(cmd, "BLINK", g_effect_base_mask);
    }
    return 1U;
  }

  if (cmd == 'e' || cmd == 'E')
  {
    g_effect_mode = LED_EFFECT_BREATH;
    if ((g_static_mask & 0x07U) != 0U)
    {
      g_effect_base_mask = g_static_mask & 0x07U;
    }
    else if ((g_effect_base_mask & 0x07U) == 0U)
    {
      g_effect_base_mask = 0x07U;
    }
    g_last_cmd = cmd;
    g_effect_last_tick = HAL_GetTick();
    apply_led_mask(g_effect_base_mask);
    g_swd_mailbox.seq++;
    if (send_ack != 0U)
    {
      send_uart_ack(cmd, "BREATH", g_effect_base_mask);
    }
    return 1U;
  }

  uint8_t mask = 0xFFU;
  if (cmd >= '0' && cmd <= '7')
  {
    mask = (uint8_t)(cmd - '0');
  }
  else if (cmd == 'r' || cmd == 'R')
  {
    mask = 0x01U;
  }
  else if (cmd == 'g' || cmd == 'G')
  {
    mask = 0x02U;
  }
  else if (cmd == 'b' || cmd == 'B')
  {
    mask = 0x04U;
  }
  else if (cmd == 'y' || cmd == 'Y')
  {
    mask = 0x03U;
  }
  else if (cmd == 'p' || cmd == 'P' || cmd == 'm' || cmd == 'M')
  {
    mask = 0x05U;
  }
  else if (cmd == 'c' || cmd == 'C')
  {
    mask = 0x06U;
  }
  else if (cmd == 'w' || cmd == 'W')
  {
    mask = 0x07U;
  }
  else if (cmd == 'o' || cmd == 'O')
  {
    mask = 0x00U;
  }

  if (mask != 0xFFU)
  {
    g_effect_mode = LED_EFFECT_STATIC;
    g_static_mask = mask & 0x07U;
    if (g_static_mask != 0U)
    {
      g_effect_base_mask = g_static_mask;
    }
    g_last_cmd = cmd;
    apply_led_mask(g_static_mask);
    g_swd_mailbox.seq++;
    if (send_ack != 0U)
    {
      send_uart_ack(cmd, mask_color_name(g_static_mask), g_static_mask);
    }
    return 1U;
  }

  return 0U;
}

static void init_cycle_counter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_breath_pwm_ticks = SystemCoreClock / 1000U;
  if (g_breath_pwm_ticks == 0U)
  {
    g_breath_pwm_ticks = 72000U;
  }
}

static void update_dynamic_effect(void)
{
  if (g_effect_mode == LED_EFFECT_STATIC)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  if (g_effect_mode == LED_EFFECT_FLOW)
  {
    if ((now - g_effect_last_tick) >= FLOW_INTERVAL_MS)
    {
      static const uint8_t kFlowMasks[6] = {0x01U, 0x03U, 0x02U, 0x06U, 0x04U, 0x05U};
      g_effect_last_tick = now;
      g_flow_step = (uint8_t)((g_flow_step + 1U) % 6U);
      g_effect_base_mask = kFlowMasks[g_flow_step];
      apply_led_mask(g_effect_base_mask);
    }
  }
  else if (g_effect_mode == LED_EFFECT_BLINK)
  {
    if ((now - g_effect_last_tick) >= BLINK_INTERVAL_MS)
    {
      g_effect_last_tick = now;
      g_blink_on = (uint8_t)(!g_blink_on);
      apply_led_mask((g_blink_on != 0U) ? g_effect_base_mask : 0x00U);
    }
  }
  else if (g_effect_mode == LED_EFFECT_BREATH)
  {
    uint32_t phase = (now - g_effect_last_tick) % BREATH_PERIOD_MS;
    uint32_t half = BREATH_PERIOD_MS / 2U;
    uint32_t linear = (phase < half)
                          ? ((phase * BREATH_PWM_STEPS) / half)
                          : (((BREATH_PERIOD_MS - phase) * BREATH_PWM_STEPS) / half);
    /* Quadratic gamma curve for natural perceived brightness */
    uint32_t duty = (linear * linear) / BREATH_PWM_STEPS;
    uint32_t pwm_pos = ((DWT->CYCCNT % g_breath_pwm_ticks) * BREATH_PWM_STEPS) / g_breath_pwm_ticks;
    uint8_t on_mask = (pwm_pos < duty) ? g_effect_base_mask : 0x00U;
    if (on_mask != g_current_mask)
    {
      write_led_pins(on_mask);
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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  init_cycle_counter();

  g_swd_mailbox.cmd = 0U;
  g_swd_mailbox.seq = 0U;
  apply_led_mask(0x00U);
  __DMB();
  g_swd_mailbox.magic = SWD_MAILBOX_MAGIC;

  /* Boot self-check: quick R -> G -> B flash, then turn all off */
  apply_led_mask(0x01U);
  HAL_Delay(80);
  apply_led_mask(0x02U);
  HAL_Delay(80);
  apply_led_mask(0x04U);
  HAL_Delay(80);
  apply_led_mask(0x00U);

  send_uart_banner();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Poll SWD SRAM mailbox (0x20000004) for zero-halt terminal commands */
    uint32_t swd_cmd_word = g_swd_mailbox.cmd;
    if (swd_cmd_word != 0U)
    {
      uint8_t swd_cmd = (uint8_t)(swd_cmd_word & 0xFFU);
      (void)handle_command_byte(swd_cmd, 0U);
      __DMB();
      g_swd_mailbox.cmd = 0U;
    }

    /* 2. Drain interrupt-driven USART1 RX ring buffer (PA9/PA10, 115200 8N1) */
    uint8_t rx_byte = 0U;
    while (USART1_TryReadByte(&rx_byte) != 0U)
    {
      (void)handle_command_byte(rx_byte, 1U);
    }

    /* 3. Advance non-blocking dynamic lighting effects (flow / blink / breath) */
    update_dynamic_effect();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSE 8MHz * PLL 9 = 72MHz, fallback to HSI 64MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Fallback to internal HSI / 2 * 16 = 64MHz if external crystal fails */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
