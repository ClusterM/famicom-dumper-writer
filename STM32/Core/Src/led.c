#include "main.h"
#include <inttypes.h>

extern TIM_HandleTypeDef htim5;
static uint8_t pwm_values[8 * 3 + 1];
static uint32_t idle_timer_start = 0;
static uint32_t last_led_update_time = 0;
static double idle_hue = 240;
static double idle_hue_speed = 0.01;
static double idle_value = 0.5;

typedef struct
{
  double r;       // a fraction between 0 and 1
  double g;       // a fraction between 0 and 1
  double b;       // a fraction between 0 and 1
} rgb;

typedef struct
{
  double h;       // angle in degrees
  double s;       // a fraction between 0 and 1
  double v;       // a fraction between 0 and 1
} hsv;

static rgb hsv2rgb(hsv in)
{
  double hh, p, q, t, ff;
  long i;
  rgb out;
  if (in.s <= 0.0)
  {       // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }
  hh = in.h;
  while (hh > 360)
    hh -= 360;
  hh /= 60.0;
  i = (long) hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i)
  {
  case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

  case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
  case 5:
  default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
  }
  return out;
}

static void update_led()
{
  while (last_led_update_time + 2 > HAL_GetTick())
  {
  }
  last_led_update_time = HAL_GetTick();
  htim5.Instance->CR1 &= ~TIM_CR1_CEN;
  htim5.Instance->CCR3 = 0;
  htim5.Instance->CNT = 0;
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, (void*) pwm_values, sizeof(pwm_values));
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t i;
  const uint8_t led = 0;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + i] = ((r >> (7 - i)) & 1) ? 67 : 22;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 8 + i] = ((g >> (7 - i)) & 1) ? 67 : 22;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 16 + i] = ((b >> (7 - i)) & 1) ? 67 : 22;
  update_led();
}

void led_off()
{
  set_led_color(0x00, 0x00, 0x00);
  idle_timer_start = HAL_GetTick();
  idle_value = 0;
}

void led_green()
{
  set_led_color(0x00, 0x80, 0x00);
}

void led_red()
{
  set_led_color(0x80, 0x00, 0x00);
}

void led_yellow()
{
  set_led_color(0x80, 0x80, 0x00);
}

void led_blue()
{
  set_led_color(0x00, 0x00, 0x40);
}

void led_magenta()
{
  set_led_color(0x80, 0x00, 0x80);
}

void led_idle()
{
  if (idle_timer_start + 5000 < HAL_GetTick())
  {
    hsv h;
    h.h = idle_hue;
    h.s = 1;
    h.v = idle_value;
    rgb r = hsv2rgb(h);
    set_led_color(r.r * 0xFF, r.g * 0xFF, r.b * 0xFF);
    idle_hue += idle_hue_speed;
    if (idle_hue > 260 || idle_hue < 220)
      idle_hue_speed *= -1;
    if (idle_value < 0.5)
      idle_value += 0.0001;
  }
}
