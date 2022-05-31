/*
 * threads.c
 *
 *  Created on: May 17, 2022
 *      Author: ainna
 */
#include "threads.h"
#include "main.h"
#include "utils.h"
/**************************************************************************************
 *                                                                                    *
 * Function:  control_threads                                            	  		  *
 * --------------------                                                               *
 *  Controls that the COMMS thread or ADCS thread has not been killed by error	      *
 *	If for some reason some of them were killed, they are created again.			  *																				  *
 *  comms, adcs: The ID of the threads to control									  *
 *																					  *
 *  returns: nothing							      								  *
 *  		 																		  *
 **************************************************************************************/
void control_threads(osThreadId_t comms, osThreadId_t adcs) {
	osThreadState_t state_comms;
	osThreadState_t state_adcs;
	state_comms = osThreadGetState(comms);
	state_adcs = osThreadGetState(adcs);
	if (state_comms == osThreadTerminated) {
		comms = osThreadNew(StartCOMMS, NULL, &COMMS_attributes);
	}
	if (state_adcs == osThreadTerminated) {
		adcs = osThreadNew(StartADCS, NULL, &ADCS_attributes);
	}
}

void obc_thread(osThreadId_t COMMSHandle, osThreadId_t ADCSHandle) {

	uint8_t currentState, exit_low, nominal, low, critical;
//	Write_Flash(CURRENT_STATE_ADDR, INIT, 1);
//	Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
//	Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
	// Signals related to NOTIFICATIONS
	uint32_t signal_received = 0;
	uint32_t signals_to_wait = CONTINGENCY_NOTI | WAKEUP_NOTI | RESET_NOTI
			| GS_NOTI | NOTGS_NOTI | TAKEPHOTO_NOTI | TAKERF_NOTI
			| DONEPHOTO_NOTI | DONERF_NOTI | PAYLOAD_ERROR_NOTI
			| EXITLOWPOWER_NOTI | SETTIME_NOTI | NOMINAL_NOTI | LOW_NOTI
			| CRITICAL_NOTI | CTEKP_NOTI | TLE_NOTI | GYRORES_NOTI
			| CALIBRATION_NOTI | POINTING_NOTI | DETUMBLING_NOTI
			| POINTING_DONE_NOTI | DETUMBLING_NOTI | ROTATE_NOTI
			| STOP_POINTING_NOTI;

	uint32_t settime, realTime, payload_time;

	for (;;) {
		// START OF THE OBC STATE MACHINE;
		// Look for a new notification (UPDATE VALUES or NOTIFY another Task)
		// function --> notifications() ?
		signal_received = osThreadFlagsWait(signals_to_wait, osFlagsWaitAny,
				10); //Mirar el temps despera
		if (signal_received & PAYLOAD_ERROR_NOTI) {
			// Notify COMMS that PL Camera is not working
			osThreadFlagsSet(COMMSHandle, PAYLOAD_ERROR_NOTI);
		}
		if (signal_received & NOMINAL_NOTI) {
			//If GS changed the NOMINAL threshold
			Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
		}
		if (signal_received & LOW_NOTI) {
			//If GS changed the LOW threshold
			Read_Flash(LOW_ADDR, &low, sizeof(low));
		}
		if (signal_received & CRITICAL_NOTI) {
			//If GS changed the CRITICAL threshold
			Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));
		}
		if (signal_received & SETTIME_NOTI) {
			//If GS send TIME
			Read_Flash(SET_TIME_ADDR, &settime, sizeof(settime));
			UnixToHumanTime(settime, &hrtc);
		}
		if (signal_received & GS_NOTI) {
			//If contact region with GS -> NOTIFY COMMS
			osThreadFlagsSet(COMMSHandle, GS_NOTI);
		}
		if (signal_received & TAKEPHOTO_NOTI) {
			// If contact TAKEPHOTO_NOTI
			// Read from memory the when to take the photo
			Read_Flash(PL_TIME_ADDR, &payload_time, sizeof(payload_time));
			// Check real time
			do {
				HumanToUnixTime(&hrtc, realTime);
			} while (payload_time > realTime + 10);
			// HOW MUCH TIME DOES IT TAKE TO POINT ?

			// Notify ADCS it should start pointing
			osThreadFlagsSet(ADCSHandle, POINTING_NOTI);
			// Wake up PL Task
			//osThreadFlagsSet(PLHandle, WAKEUP_NOTI);
		}
		if (signal_received & TAKERF_NOTI) {
			// Wake up PL RF Task
			// osThreadFlagsSet("Task PAYLOAD RF", WAKEUP_NOTI);
		}
		if (signal_received & DONEPHOTO_NOTI) {
			// Notify ADCS to stop pointing
			osThreadFlagsSet(ADCSHandle, STOP_POINTING_NOTI);
			// Notify COMMS
			osThreadFlagsSet(COMMSHandle, DONEPHOTO_NOTI);
		}
		if (signal_received & DONERF_NOTI) {
			// Notify COMMS
			// osThreadFlagsSet(COMMSHandle, DONERF_NOTI);
		}
		if (signal_received & CTEKP_NOTI) {
			// Notify ADCS it should start pointing
			osThreadFlagsSet(ADCSHandle, CTEKP_NOTI);
		}
		if (signal_received & TLE_NOTI) {
			// Notify ADCS it should start pointing
			osThreadFlagsSet(ADCSHandle, TLE_NOTI);
		}
		if (signal_received & GYRORES_NOTI) {
			// Notify ADCS it should start pointing
			osThreadFlagsSet(ADCSHandle, GYRORES_NOTI);
		}
		if (signal_received & CALIBRATION_NOTI) {
			// Notify ADCS it should start pointing
			osThreadFlagsSet(ADCSHandle, CALIBRATION_NOTI);
		}

		// Check which state
		switch (currentState) {

		case CHECK: // Check satellite general state
			/* State that periodically checks the satellite general state (batteries,
			 * temperatures, voltages...
			 * It also checks that all the threads are running
			 * From this state the satellite can go to contingency states
			 * if systemstate() returns a value different than 0 */

			// It is only done the first time we enter to IDLE
			// We send notifications to COMMS, ADCS tasks to start tunning (IDLE_State_noti)
			if (system_state(&hi2c1, nominal, low, critical) > 0) {
				currentState = CONTINGENCY;
				// Only update if we change state
				Write_Flash(PREVIOUS_STATE_ADDR, CHECK, 1);
				Write_Flash(CURRENT_STATE_ADDR, &currentState,
						sizeof(currentState));
			} else {
				sensorReadings(&hi2c1); /*Updates the values of temperatures, voltages and currents*/
				control_threads(COMMSHandle, ADCSHandle);
			}
			break;
		case CONTINGENCY:
			/*Turn STM32 to Stop Mode or Standby Mode
			 *Loop to check at what batterylevel are we
			 *Out of CONTINGENCY State when batterylevel is NOMINAL
			 */
			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

			// Avisar a COMMS que entrem en CONTINGENCY (enviar paquet de telemetria)
			// S'envia cada vegada que rebem un EXITLOWPOWER_NOTI i la bateria no millora
			osThreadFlagsSet(COMMSHandle, CONTINGENCY_NOTI);
			// Esperem que COMMS rebi telecommand de la GS (EXITLOWPOWER_NOTI)
			signal_received = osThreadFlagsWait(signals_to_wait, osFlagsWaitAny, 10);
			if (signal_received & EXITLOWPOWER_NOTI) {
				// Mirem el nivell de bateria
				// Si ha empitjorat
				if (system_state(&hi2c1, nominal, low, critical) > 1) {
					currentState = SUNSAFE;
					// Actualitzem l'estat només si canviem d'estat
					Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
					Write_Flash(CURRENT_STATE_ADDR, &currentState,
							sizeof(currentState));

					// Si ha millorat
				} else if (system_state(&hi2c1, nominal, low, critical) == 0) {
					/*Return to Run Mode*/
					currentState = CHECK;
					// Actualitzem l'estat només si canviem d'estat
					Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
					Write_Flash(CURRENT_STATE_ADDR, &currentState,
							sizeof(currentState));
				}
			}

			// Una opció és fer reset total del satelit quan surti de contingency

			break;

		case SUNSAFE:
			Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
			// Entrem en el SleepMode
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

			// IWDG initialize
			if (system_state(&hi2c1, nominal, low, critical) == 2) {
				// Nothing has changed:
				// Delay the system so the IWDG resets the system
				HAL_Delay(33000);
			}
			// IWDG exits SleepMode automatically
			// system_state = 3 --> battery level < CRITICAL
			else if (system_state(&hi2c1, nominal, low, critical) == 3) {
				currentState = SURVIVAL;
				// Update only if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));

				// system_state = 1 --> LOW < battery level < NOMINAL
			} else if (system_state(&hi2c1, nominal, low, critical) == 1) {
				/*Return to Run Mode*/
				currentState = CONTINGENCY;
				// Update only if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));
			}
			break;

		case SURVIVAL:
			// Enter in the Low Power Sleep Mode
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			// Refresh IWDG to avoid reseting the system
			HAL_IWDG_Refresh(&hiwdg);
			// From SURVIVAL we can only go to CONTINGENCY
			if (system_state(&hi2c1, nominal, low, critical) == 1) {
				currentState = CONTINGENCY;
				// Only update if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SURVIVAL, 1);
				Write_Flash(CURRENT_STATE_ADDR, &currentState,
						sizeof(currentState));
			} else {
				HAL_Delay(33000);
			}

			break;

		case INIT:
			init(&hi2c1);
			Write_Flash(PREVIOUS_STATE_ADDR, INIT, 1);
			Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
			// Wake up COMMS and ADCS tasks
			osThreadFlagsSet(ADCSHandle, WAKEUP_NOTI);
			osThreadFlagsSet(COMMSHandle, WAKEUP_NOTI);
			// Send notification to ADCS --> DETUMBLING_NOTI
			osThreadFlagsSet(ADCSHandle, DETUMBLING_NOTI);

			break;
			/*If we reach this state something has gone wrong*/
		default:
			/*REBOOT THE SYSTEM*/
			break;
		}

		/*Start a TIMER*/

//	    return 0;
//todo variable que conti ticks rellotge per fer reset
	}

	void comms_thread() {
		stateMachine();
	}

	void adcs_thread() {

	}

	void pl_thread() {
// signal to wait son les possibles senyals que podem rebre
		uint32_t signal_to_wait = WAKEUP_NOTI | POINTING_DONE_NOTI
				| CONTINGENCY_NOTI;
		uint32_t signal_received = 0, payload_time, realTime;

		for (;;) {
			// Wait for Wake up notification
			osThreadFlasgsWait(WAKEUP_NOTI, osFlagsWaitAny, osWaitForever);
			// If WAKEUP
			// Initialize the Camera (SWITCH/PIN VCC) -> PC3-11

			// Wait for ADCS to point
			osThreadFlasgsWait(POINTING_DONE_NOTI, osFlagsWaitAny,
					osWaitForever);
			// Wait until the it's time
			do {
				// Check real time
				HumanToUnixTime(&hrtc, realTime);

			} while (payload_time > realTime + 10); // TODO: x ?

			// Take Data
			//while (!takePhoto(&huart1));
			// resetCommsParams();
			// If photo has been taken correctly:
			osThreadFlasgsSet(OBCHandle, DONEPHOTO_NOTI);

			// Turn off the CAMERA
		}
	}
}
