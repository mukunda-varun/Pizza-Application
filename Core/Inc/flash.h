/*
 * flash.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Varun
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"

#define BANK2_FLASH_SECTOR_7  0x080E0000


void read_production_report(uint32_t* flash_addr,uint8_t* read_data,uint32_t size);
uint8_t FLASH_Write(uint32_t* flash_addr ,uint32_t *data,uint32_t data_length);


#endif /* INC_FLASH_H_ */
