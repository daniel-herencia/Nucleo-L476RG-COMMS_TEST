/*
 * notifications.h
 *
 *  Created on: 29 may. 2022
 *      Author: Usuari
 */

#ifndef INC_NOTIFICATIONS_H_
#define INC_NOTIFICATIONS_H_

//GENERAL NOTIFICATIONS
#define WAKEUP_NOTIFICATION //Wake up the notification
#define SETTIME_NOTIFICATION //Indicates to OBC the time
#define CONTINGENCY_NOTIFICATION  //Indicates to OBC to go to contingency state

//COMMS NOTIFICATIONS
#define EXITLOWPOWER_NOTIFICATION //Indicates to OBC to leave Comms LOWPOWER state

//PAYLOAD NOTIFICATIONS
#define TAKEPHOTO_NOTIFICATION  //Indicates to OBC to take a photo to camera
#define DONEPHOTO_NOTIFICATION  //Photo done, camera go to sleep info to OBC
#define TAKERF_NOTIFICATION //Indicates to the OBC the antenna info
#define DONERF_NOTIFICATION //Indicates to the OBC antenna info done

//ADCS NOTIFICATIONS
#define NOMINAL_NOTIFICATION //Indicates to the OBC ADCS info
#define LOW_NOTIFICATION //Indicates to the OBC ADCS info
#define CRITICAL_NOTIFICATION //Indicates to the OBC ADCS info
#define SETCONSTANT_NOTIFICATION //Indicates to the OBC ADCS info
#define TLE_NOTIFICATION //Indicates to the OBC ADCS info
#define SETGYRO_NOTIFICATION //Indicates to the OBC ADCS info
#define SENDCALIBRATION_NOTIFICATION //Indicates to the OBC ADCS info

#endif /* INC_NOTIFICATIONS_H_ */
