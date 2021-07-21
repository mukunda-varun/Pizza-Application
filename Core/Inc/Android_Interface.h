/*
 * Android_Interface.h
 *
 *  Created on: 14-Oct-2020
 *      Author: Supriya
 */

#ifndef INC_ANDROID_INTERFACE_H_
#define INC_ANDROID_INTERFACE_H_
//#include "usbd_cdc_if.h"

/*Added on 17-03-21*/
#define   		DAILY_MAINTENANCE_STATE  			32
#define			SERVICE_FLOUR_MEASUREMENT   		33
#define   		SERVICE_WATER_MEASUREMENT			34
#define 		SERVICE_OIL_MEASUREMENT				35
#define 		SERVICE_PRESS_MEASUREMENT			36
#define			MAINTENANCE_ABORT					15
/*Added on 17-03-21*/

typedef enum
{
   HANDSHAKE=1,
   QUANTITY_UPDATE,//2
   MIXING_INIT,//3
   KNEADING_COATING,//4
   DR_ID,//5
   DPE,//6
   PB_ID,//7
   START,//10
   ABORT,//11
   STATUS,//12 ,
   CM_SETTING,//8
   HW_SETTING_1,
   HW_SETTING_2,
   HW_SETTING,//9
   //Added on 15-Feb
//   SERVICE_ABORT,
   //Added on 09-Mar
}msg_id_t;

typedef enum
{
   MAINTENANCE_FLOUR_PRIMING = 1,
   MAINTENANCE_WATER_PRIMING,
   MAINTENANCE_OIL_PRIMING,
   MAINTENANCE_WATER_CLEANING,
   MAINTENANCE_OIL_CLEANING,
   KNEADER_CLEANING_TOP,
   KNEADER_CLEANING_POS,
   KNEADER_CLEANING_BTM,
   EJECTOR_CLEANING_START,		//Home
   EJECTOR_CLEANING_END,		//Front End
   PRESS_CLEANING_BTM,
   PRESS_CLEANING_TOP,
//   MAINTENANCE_ABORT,
}maintenance;

extern uint8_t process_start;
extern void  Update_Status(int i);
void Update_Quantity_Sufficiency(void);
//extern int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
extern void  Send_ADR_Response(uint8_t id, uint8_t data);
void Update_end_of_process_status(void);
extern void StartAndroidProcessingTask(void *argument);
#endif /* INC_ANDROID_INTERFACE_H_ */
