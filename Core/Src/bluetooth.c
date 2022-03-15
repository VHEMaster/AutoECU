/*
 * bluetooth.c
 *
 *  Created on: 15 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "bluetooth.h"
#include "xCommand.h"
#include "delay.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static eTransChannels btChannel = etrNone;
static char btName[32];
static uint32_t btPin = 1234;
static volatile uint8_t btEnable = 0;

static GPIO_TypeDef *btRstPort = NULL;
static uint16_t btRstPin = 0;
static GPIO_TypeDef *btPwrPort = NULL;
static uint16_t btPwrPin = 0;
static GPIO_TypeDef *btKeyPort = NULL;
static uint16_t btKeyPin = 0;
static GPIO_TypeDef *btLedPort = NULL;
static uint16_t btLedPin = 0;

static int32_t bt_send(const char *msg, ...) {
  static char buffer[MAX_PACK_LEN];
  static uint32_t len;
  va_list args;

  va_start(args, msg);
  vsnprintf(buffer, sizeof(buffer) - 1, msg, args);
  buffer[MAX_PACK_LEN - 1] = '\0';
  va_end(args);
  len = strlen(buffer);
  xSenderRaw(btChannel, (uint8_t *)msg, len);

  return 1;
}

void bluetooth_init(eTransChannels channel)
{
  btChannel = channel;
}

void bluetooth_register_pwr_pin(GPIO_TypeDef *port, uint16_t pin)
{
  btPwrPort = port;
  btPwrPin = pin;
}

void bluetooth_register_rst_pin(GPIO_TypeDef *port, uint16_t pin)
{
  btRstPort = port;
  btRstPin = pin;
}

void bluetooth_register_key_pin(GPIO_TypeDef *port, uint16_t pin)
{
  btKeyPort = port;
  btKeyPin = pin;
}

void bluetooth_register_led_pin(GPIO_TypeDef *port, uint16_t pin)
{
  btLedPort = port;
  btLedPin = pin;
}

void bluetooth_loop(void)
{
  static uint32_t last_time;
  static uint8_t state = 0;
  int8_t bt_status = 0;
  uint32_t now = Delay_Tick;

  if(!btEnable) {
    state = 0;
    HAL_GPIO_WritePin(btKeyPort, btKeyPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(btRstPort, btRstPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(btPwrPort, btPwrPin, GPIO_PIN_SET);
  } else {

    switch(state) {
      case 0 :
        HAL_GPIO_WritePin(btKeyPort, btKeyPin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(btRstPort, btRstPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(btPwrPort, btPwrPin, GPIO_PIN_RESET);
        last_time = now;
        state++;
        break;
      case 1 :
      case 3 :
      case 5 :
      case 7 :
      case 9 :
      case 11 :
      case 13 :
      case 15 :
      case 17 :
      case 19 :
      case 21 :
        if(DelayDiff(last_time, now) > 500) {
          state++;
        }
        break;
      case 2: HAL_GPIO_WritePin(btRstPort, btRstPin, GPIO_PIN_SET); last_time = now; state++; break;
      case 4: if((bt_status = bt_send("AT+RESET\r\n"))) { last_time = now; state++; } break;
      case 6: if((bt_status = bt_send("AT+ORGL\r\n"))) { last_time = now; state++; } break;
      case 8: if((bt_status = bt_send("AT+ROLE=0\r\n"))) { last_time = now; state++; } break;
      case 10: if((bt_status = bt_send("AT+NAME=%s\r\n", btName))) { last_time = now; state++; } break;
      case 12: if((bt_status = bt_send("AT+CMODE=1\r\n"))) { last_time = now; state++; } break;
      case 14: if((bt_status = bt_send("AT+PSWD=%04d\r\n", btPin))) { last_time = now; state++; } break;
      case 16: if((bt_status = bt_send("AT+UART=38400,0,0\r\n"))) { last_time = now; state++; } break;
      case 18: HAL_GPIO_WritePin(btKeyPort, btKeyPin, GPIO_PIN_RESET); last_time = now; state++; break;
      case 20: if((bt_status = bt_send("AT+RESET\r\n"))) { last_time = now; state++; } break;
      case 22: break;
      default:
        state = 0;
        break;
    }
  }
}

void bluetooth_enable(const char *name, uint32_t pin)
{
  strcpy(btName, name);
  btPin = pin;
  btEnable = 1;
}

void bluetooth_disable(void)
{
  btEnable = 0;
}
