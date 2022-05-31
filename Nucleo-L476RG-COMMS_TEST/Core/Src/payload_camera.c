

#include <payload_camera.h>
#include <flash.h>
#include <definitions.h>

//VARIABLES
uint8_t dataBuffer[600], bufferLength;
uint16_t frameLength;

uint8_t commInit[2] = {0x56, 0x00};
uint8_t commCapture = 0x36;
uint16_t bSize = 512;
uint8_t count=0;
uint8_t start_pos=0;
uint8_t error_type=0;

//COMANDOS
uint8_t resetCmd[1] = {0x00};
uint8_t captureImageCmd[2] = {0x01, 0x00};
uint8_t stopCaptureCmd[2] = {0x01, 0x03};
uint8_t setCompressibilityCmd[6] = {0x05, 0x01, 0x01, 0x12, 0x04, 0xFF}; //FF is max. compression and 00 is min. compression
uint8_t setResolutionCmd[6] = {0x05, 0x04, 0x01, 0x00, 0x19, 0x22};
uint8_t setBandRateCmd[7] = {0x06, 0x04, 0x02, 0x00, 0x08, 0x0D, 0xA6};

//ACKs
uint8_t check[4]={0x76, 0x00, 0x00, 0x00};
uint8_t ACKSetComp[1] = {0x00};
uint8_t ACKSetResol[1] = {0x00};
uint8_t ACKGetCap[1] = {0x00};
uint8_t ACKReadIDLength[3] = {0x04, 0x00, 0x00};
uint8_t ACKStopCap[1] = {0x00};
uint8_t ACKImage1[4]={0xFF, 0xD8, 0xFF, 0xD9};


//NOT FINISHED
//uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts, uint8_t ACK){
//  int i = 0;
//  bufferLength = 0;
//  while (attempts != i && bufferLength != expLength)//si attempts == i o si bufferLenght == expLength sale del while
//  {
////    if ()//mirar si tenemos algo que leer
////    { // Is there any data?
////      delay(1);
////      i++;
////      continue;
////    }
//    i = 0;
//    bufferLength++;
//    //dataBuffer[bufferLength++] = swsInstance.read(); // And we fill it with data
//	//for(int i = 0; i<)
//    HAL_UART_Receive(huart, dataBuffer, expLength, 100);
//  }

	//TODO hacer que espere hasta que reciba algo
	//HAL_UART_Receive(huart, dataBuffer, expLength, 100); //Recibimos los datos

	//if(ACK[0]!=0x00){ //Si los datos que recibimos son un ACK
	//	for(int i=0; i<sizeof(dataBuffer); i++){ //Comparar si lo que hemos recibido es el ACK que esperamos.
	//		if(dataBuffer[i]!=ACK[i]){
	//			return false; //Si no lo es, devolvemos false
	//		}
	//	}
	//}


 // return expLength;
//}

bool runCommand(UART_HandleTypeDef huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint16_t expLength)
{ // Flushes the buffer, sends the command and hexData then checks and verifies it

	for(int attempt=0; attempt<NUM_ATTEMPTS; attempt++){ //We have 3 attempts to send and receive the data correctly
	// Flush the reciever buffer
		//readResponse(huart, 100, 10);
	  memset(dataBuffer,0,sizeof(dataBuffer));

	  // Send the data
	  HAL_UART_Transmit(&huart, (uint8_t *)commInit, 2, HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart, &command, 1, HAL_MAX_DELAY);

	  for (int i = 0; i < dataArrayLength; i++){
		  HAL_UART_Transmit(&huart, &hexData[i], 1, HAL_MAX_DELAY);
	  }

	  // Check the data
	  //if (readResponse(huart, expLength, 100, ACK) != expLength)
	  //{
	  //  return false;
	  //}

	  //Data should always be 76, 00, command, 00; ES UN BOOL
	  //return dataBuffer[0] == 0x76 &&
	  //	     dataBuffer[1] == 0x0 &&
	  //	     dataBuffer[2] == command &&
	  //	     dataBuffer[3] == 0x0;

	  if (HAL_UART_Receive(&huart, dataBuffer, expLength, HAL_MAX_DELAY) == 0x03){ //If it returns 0x03, it means there is a timeout, so the connection is lost
		  error_type = 1; //This is the first type of error
		  return false;
	  } //We receive the expected bytes of data
	  //}

	  if(checkACK(command, expLength)==true){ //If the ACK received is the expected one, we return true
		  return true;
	  }else if (attempt==(NUM_ATTEMPTS-1) && checkACK(command, expLength)==false){ //If on the third attempt we are still not receiving the expected ACK, we return false
		  error_type = 2; //This is the second type of error
		  return false;
	  }

	  //return true;
	}

}

bool reset(UART_HandleTypeDef huart){
	return runCommand(huart, 0x26, resetCmd, sizeof(resetCmd), 58); //It is 58 because it sends information about the camera after the ACK
}

bool captureImage(UART_HandleTypeDef huart){
	return runCommand(huart, 0x36, captureImageCmd, sizeof(captureImageCmd), 5);
}

bool stopCapture(UART_HandleTypeDef huart){
	return runCommand(huart, 0x36, stopCaptureCmd, sizeof(stopCaptureCmd), 5);
}

bool setCompressibility(UART_HandleTypeDef huart){
	//TODO the compressibility should be a telecomand and must be extracted from the memory of the STM32, talk with OBC to agree where will be stored this information
	//If the compressibility variable is stored in memory, is not needed to give it as a parameter to the function, use Read_Flash function;
	//DONE

	//Read_Flash(PHOTO_COMPRESSION_ADDR,&compressibility,sizeof(compressibility));

	//setCompressibilityCmd[5] = compressibility; //The value should be obtained from memory
	return runCommand(huart, 0x31, setCompressibilityCmd, sizeof(setCompressibilityCmd), 5);
}

bool setResolution(UART_HandleTypeDef huart){
	//the resolution should be a telecomand and must be extracted from the memory of the STM32
	//There is a resolution condition which will only be used if we the OBC wants to change the resolution
	//uint8_t resolution = MIN_RES;
	uint8_t resolution;

	//if(resolution_con == MAX_RES_CON){
		Read_Flash(PHOTO_RESOL_ADDR,&resolution,sizeof(resolution));
	//}

	setResolutionCmd[5] = resolution;

	return runCommand(huart, 0x31, setResolutionCmd, sizeof(setResolutionCmd), 5);
}

bool getFrameLength(UART_HandleTypeDef huart)
{ // ~ Get frame length
	if (runCommand(huart, 0x34, captureImageCmd, sizeof(captureImageCmd), 9)) //I use the captureImageCmd because it's the same as for the data length
  {
	  frameLength = dataBuffer[5]; //Recreating split hex numbers from 4 bytes
	  frameLength <<= 8;
	  frameLength |= dataBuffer[6];
	  frameLength <<= 8;
	  frameLength |= dataBuffer[7];
	  frameLength <<= 8;
	  frameLength |= dataBuffer[8];

	  //uint8_t length_vect[2]={0};
	  //length_vect[1]= dataBuffer[7];
	  //length_vect[2]=dataBuffer[8];
	  //Write_Flash(PHOTO_LENGTH_ADDR, length_vect, (uint16_t)sizeof(length_vect));
	  return true;
  }

  return false;
}

bool retrieveImage(UART_HandleTypeDef huart)
{ // * Retrieve photo data

	uint16_t size = frameLength;
	uint8_t dataVect[size]; //We create dataVect[], an array that will store the image data
	uint16_t toRead; //We create toRead, which will have the number of bytes that will be sent on each packet
	uint32_t framePointer=0; //We create framePointer, a variable which will have the position of the dataVect were the incoming data will be stored
	//todo errase this bucle. Use the memset function (search this function on the internet)
	//for(int i = 0; i < frameLength; i++){
	//  dataVect[i] = 0;
	//}
	//done

	memset(dataVect,0, sizeof(dataVect)); //We initialize it to 0.

	while (frameLength > 0) //While there is data to read
	{

		 toRead = min(bSize, frameLength); // Bytes read each loop
												// >> Bitwise, desplaza 8 bits en ese caso

		uint8_t hexData[] = {0x0C, 0x0, 0x0A, 0x0, 0x0,
							 (framePointer >> 8) & 0xFF, framePointer & 0xFF, 0x0, 0x0,
							 (toRead >> 8) & 0xFF, toRead & 0xFF, 0x0, 0x0A
							}; //We are asking for photo blocks of 512 Bytes each iteration

		//Here we ask for the data
		if(!runCommand(huart, 0x32, hexData, sizeof(hexData), 2*(5)+toRead))
		{
			return false;
		}

		//We fill the dataVect with the bytes received
		for(int i = 0; i < toRead; i++){
			dataVect[framePointer+i] = dataBuffer[i+5];
		}

		framePointer += toRead;
		frameLength -= toRead;
	}

	//We save the data in the Flash memory
	Flash_Write_Data(PHOTO_ADDR, dataVect, (uint16_t)(sizeof(dataVect)/8+1));
	return true;
}

bool takePhoto(UART_HandleTypeDef huart){
	//todo create a protocol in order to handle errors in the communication
	//takePhoto
	if(captureImage(huart)){

		//actualize frameLength
		if(getFrameLength(huart)){

			//saves the image to the flash mem
			if(retrieveImage(huart)){

				//stops capture
				if(stopCapture(huart)){
					return true;
				}
			}
		}
	}

	return false;
}

bool powerCam(UART_HandleTypeDef huart){

	//if(//Function to activate pin PC3-11){
	//HAL_Delay(2500);
	//	return true
	//}

	//return false;

}

bool initCam(UART_HandleTypeDef huart){

	if(reset(huart)){
		//HAL_Delay(1000);
		if(setCompressibility(huart)){
			if(setResolution(huart)){
				return true;
			}
		}
	}

	return false;
	//setResolution(huart);
}

uint16_t min(uint16_t x, uint16_t y)
{
  return (x < y) ? x : y;
}

bool errorProt(bool (*f)(UART_HandleTypeDef), UART_HandleTypeDef huart){

	if(error_type==2){
		if(initCam(huart)){
			if((*f)(huart)){
				return true;
			}/*else{
				TODO Switch off the switch that gives power (pin PC3-11) and execute powerCam()
				if(switch off and powerCam() return true){
					if(initCam(huart)){
						if((*f)(huart)){
							return true;
						}
					}
				}
			}*/
		}
	}

	//If it arrives here, we need to send a telecommand to the GS
	//xTaskNoitfy("Task OBC", PAYLOAD_ERROR_NOTI, eSetBits);
	return false;
}

bool checkACK(uint8_t command, uint16_t expLength){ //Checks if the received data is the expected one

	check[2] = command;


	for(int j=0; j<sizeof(dataBuffer); j++){ //We iterate through the dataBuffer to find where the ACK starts
		if(dataBuffer[j]==0x76){
			start_pos = j; //We store the position where the ACK starts
			break; //We exit the loop
		}
	}

	//Here we check if the first 4 positions match with the ones expected
	for(int i=0; i<sizeof(check); i++){
		if(dataBuffer[start_pos+i]!=check[i]){
			return false;
		}
	}

	//Now, we particularize the remaining bytes for each type of command
	if(command==0x36 || command==0x31){
		if(dataBuffer[start_pos+4]!=0x00){ //Check if the 5th position value corresponds with the expected one
			return false;
		}
	} else if (command==0x34){ //dataLength command
		if(dataBuffer[start_pos+4]!=0x04 || dataBuffer[start_pos+5]!=0x00 || dataBuffer[start_pos+6]!=0x0){ //Check if the 3 following bytes values are the ones expected
			return false;
		} else if(dataBuffer[start_pos+7]==0 && dataBuffer[start_pos+8]==0){ //In case bytes 8 and 9 are empty (they should have the data length), that would mean there is no photo or something has gone wrong
			return false;
		}
	} else if (command==0x32){ //retrieveImage command
		if(dataBuffer[start_pos+4]!=0x00 || dataBuffer[start_pos+expLength-1]!=0x00){ //As it has 1 ACK at the beginning and another one at the end, we check them
			return false;
		}
		for (int i=expLength-5; i<expLength-1; i++){
			if(dataBuffer[start_pos+i]!=check[i-(expLength-5)]){
				return false;
			}
		}
	}

	//If the function arrives here, that will mean everything has gone correctly, so we will send a 1
	return true;
}
