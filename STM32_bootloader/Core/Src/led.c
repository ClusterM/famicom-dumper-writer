#include "main.h"
#include <inttypes.h>

extern TIM_HandleTypeDef htim5;
static uint8_t pwm_values[8 * 3 + 1];
static uint32_t last_led_update_time = 0;

static void update_led()
{
  while (last_led_update_time + 2 > HAL_GetTick())
  {
  }
  HAL_TIMEx_PWMN_Stop_DMA(&htim5, TIM_CHANNEL_3);
  htim5.Instance->CCR3 = 1;
  htim5.Instance->CNT = 0;
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, (void*) pwm_values, sizeof(pwm_values));
  last_led_update_time = HAL_GetTick();
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t i;
  const uint8_t led = 0;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + i] = ((r >> (7 - i)) & 1) ? 65 : 24;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 8 + i] = ((g >> (7 - i)) & 1) ? 65 : 24;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 16 + i] = ((b >> (7 - i)) & 1) ? 65 : 24;
  pwm_values[sizeof(pwm_values) - 1] = 0;
  update_led();
}

void led_off()
{
  set_led_color(0x00, 0x00, 0x00);
}

void led_white()
{
  set_led_color(0xFF, 0xFF, 0xFF);
}

void led_green()
{
  set_led_color(0x00, 0xFF, 0x00);
}

void led_red()
{
  set_led_color(0xFF, 0x00, 0x00);
}

void led_yellow()
{
  set_led_color(0xFF, 0xFF, 0x00);
}

void led_blue()
{
  set_led_color(0x00, 0x00, 0xFF);
}

void led_magenta()
{
  set_led_color(0xFF, 0x00, 0xFF);
}

void led_cyan()
{
  set_led_color(0x00, 0xFF, 0xFF);
}
