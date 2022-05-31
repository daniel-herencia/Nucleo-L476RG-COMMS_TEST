
#ifndef INC_THREADS_H_
#define INC_THREADS_H_

#include "cmsis_os.h"
#include "configuration.h"

/* Definitions for OBC */
extern osThreadId_t OBCHandle;
extern const osThreadAttr_t OBC_attributes;
// = {
//  .name = "OBC",
//  .stack_size = 4048 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
/* Definitions for COMMS */
extern osThreadId_t COMMSHandle;
extern const osThreadAttr_t COMMS_attributes;
//= {
//  .name = "COMMS",
//  .stack_size = 4048 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
/* Definitions for ADCS */
extern osThreadId_t ADCSHandle;
extern const osThreadAttr_t ADCS_attributes;
//= {
//  .name = "ADCS",
//  .stack_size = 4048 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};

void control_threads(osThreadId_t comms, osThreadId_t adcs);

void obc_thread(osThreadId_t COMMSHandle, osThreadId_t ADCSHandle);

void comms_thread();

void adcs_thread();

void pl_thread();

//plrf_thread: com es desplega l'antena ?

void StartOBC(void *argument);
void StartCOMMS(void *argument);
void StartADCS(void *argument);


#endif /* INC_THREADS_H_ */
