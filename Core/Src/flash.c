/*
 * flash.c
 *
 *  Created on: Feb 4, 2021
 *      Author: Varun
 */
#include "flash.h"

extern uint8_t report_data[100];
extern uint32_t flashdestination;
extern report_t production_report_t, local_report;
/* @brief:  Function to read data from flash
 * @params: NOTHING
 * @return: NOTHING*/
void read_production_report(uint32_t* flash_addr, uint8_t* read_data,uint32_t size)
{
	uint16_t read_cnt=0;
	do
	{
//		 *read_data++ = *(uint8_t*)(PRODUCTION_REPORT_ADDRESS + read_cnt);
		 read_cnt++;
	}while(read_cnt <size);
	memcpy(&local_report,&report_data,sizeof(report_t));
}
uint8_t FLASH_Write(uint32_t* flash_addr ,uint32_t *data,uint32_t data_length)
{
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	HAL_FLASH_OB_Unlock();

	volatile uint32_t  index=0;
	/*Erase the sector before writing the data*/
	FLASH_Erase_Sector(FLASH_SECTOR_2, FLASH_BANK_2, VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_BANK_2, VOLTAGE_RANGE_3);

	/*Erase the sector before writing the data*/

	volatile HAL_StatusTypeDef status;
	status = HAL_OK;
	while(index < data_length)
	{
		if (status == HAL_OK)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (*flash_addr), (uint32_t )data);
			if(status == HAL_OK)
			{
				*flash_addr = (*flash_addr)+32;//Destination address
				data = data+8;                      //Source address
				index+=32;
			}
		}
	}
	return 1;
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

