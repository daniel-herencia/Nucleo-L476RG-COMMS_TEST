/*
 * payload_camera.h
 *
 *  Created on: 31 may. 2022
 *      Author: Usuari
 */

#ifndef INC_PAYLOAD_CAMERA_H_
#define INC_PAYLOAD_CAMERA_H_


#include "stm32l4xx_hal.h"
#include <string.h> // Usado para la funcion memcmp
#include <stdio.h>
#include <stdbool.h> //PARA EL BOOL
#include <stdlib.h> //PARA GUARDAR DATOS
//todo hacer criba de las librerias, estraer solo las funciones utiles.

#define NUM_ATTEMPTS 3
#define MIN_RES 0x22
#define MIN_RES_CON 1
#define MAX_RES_CON 0


/**
  * Waits for data to be received and stores the information to the global variable dataBuffer
  */
//uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts, uint8_t ACK);


/**
  * Transmitts information using the USART protocol
  * reads the response using readResponse
  * Veryfy if the received data is the expected one
  */
bool runCommand(UART_HandleTypeDef huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint16_t expLength);

bool reset(UART_HandleTypeDef huart);

/**
  * #2
  * Takes photo and saves it in the camera memory
  */
bool captureImage(UART_HandleTypeDef huart);

/**
  * #3
  * Actualize the value of the variable frameLength with the new length of the image
  */
bool getFrameLength(UART_HandleTypeDef huart);

/**
  * #4
  * Saves the image to the flash memory of the STM32
  */
bool retrieveImage(UART_HandleTypeDef huart);

/**
  * #5
  * Stops the capture
  */
bool stopCapture(UART_HandleTypeDef huart);

/**
  * #6
  * Sets the compressibility of the image
  */
bool setCompressibility(UART_HandleTypeDef huart);

/**
  * #7
  * Sets the resolution of the image
  */
bool setResolution(UART_HandleTypeDef huart);

/**
  * #8
  * Executes the different commands needed to take a photo
  * captureImage
  * getFrameLength
  * retrieveImage
  * stopCapture
  */
bool takePhoto(UART_HandleTypeDef huart);

bool powerCam(UART_HandleTypeDef huart);

bool errorProt(bool (*f)(UART_HandleTypeDef), UART_HandleTypeDef huart);

bool initCam(UART_HandleTypeDef huart);

uint16_t min(uint16_t bSize, uint16_t frameLength);

bool checkACK(uint8_t command, uint16_t expLength);



#endif /* INC_PAYLOAD_CAMERA_H_ */
