/*
 * bluetooth.h
 *
 *  Created on: 15 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "xCommand.h"

void bluetooth_init(eTransChannels channel);

void bluetooth_register_pwr_pin(GPIO_TypeDef *port, uint16_t pin);
void bluetooth_register_rst_pin(GPIO_TypeDef *port, uint16_t pin);
void bluetooth_register_key_pin(GPIO_TypeDef *port, uint16_t pin);
void bluetooth_register_led_pin(GPIO_TypeDef *port, uint16_t pin);

void bluetooth_loop(void);
void bluetooth_enable(const char *name, uint32_t pin);
void bluetooth_disable(void);


#endif /* INC_BLUETOOTH_H_ */
