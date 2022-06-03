
#include "comms.h"


typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
    START_CAD,
}States_t;

typedef enum
{
    CAD_FAIL,
    CAD_SUCCESS,
    PENDING,
}CadRx_t;



uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

#if(RX_FW == TX_FW)
    #error "Please define only one firmware."
#endif
CadRx_t CadRx = CAD_FAIL;
bool PacketReceived = false;
bool RxTimeoutTimerIrqFlag = false;
uint16_t channelActivityDetectedCnt = 0;
uint16_t RxCorrectCnt = 0;
uint16_t RxErrorCnt = 0;
uint16_t RxTimeoutCnt = 0;
uint16_t SymbTimeoutCnt = 0;
int16_t RssiMoy = 0;
int8_t SnrMoy = 0;
uint16_t txCounter = 0;
uint8_t Memory[] = {'H','O','L','A'};


uint8_t tx_non_stop = 0; //1 => yes ; 0 => not
uint8_t testRX = false;



/* FLAGS */
uint8_t error_telecommand = false;
uint8_t tx_flag = false;

/* FLAGS */
uint8_t tle_telecommand = false;
uint8_t telecommand_rx = false;
uint8_t contact_GS = false; //To avoid TX beacon
uint8_t request_execution = 0;

uint8_t paquet_to_send;
uint8_t last_telecommand[BUFFER_SIZE];	//Last telecommand RX
uint8_t request_counter = 0;
uint8_t paquet_number = 0;
uint8_t window_paquet = 0;

uint16_t rx_attemps_counter = 0;	//Instead of timeout with timers, counting iterations


/* COUNTERS */
uint8_t rtx_confirms = 0;	//Maximum 3 retransmissions of execution request

// VARIABLES FROM OLD CODE
uint8_t calib_packets = 0;			//Counter of the calibration packets received
uint8_t tle_packets = 0;			//Counter of the tle packets received
uint8_t telemetry_packets = 0;		//Counter of telemetry packets sent
uint8_t count_packet[] = {0};		//To count how many packets have been sent (maximum WINDOW_SIZE)
uint8_t count_window[] = {0};		//To count the window number
uint8_t count_rtx[] = {0};			//To count the number of retransmitted packets
uint8_t i=0;						//variable for loops
uint8_t j=0;						//variable for loops
uint8_t k=0;						//variable for loops
uint64_t ack;						//Information rx in the ACK (0 => ack, 1 => nack)
uint8_t nack_number;				//Number of the current packet to retransmit
bool nack;							//True when retransmission necessary
bool full_window;					//Stop & wait => to know when we reach the limit packet of the window
bool statemach = true;				//If true, comms workflow follows the state machine. This value should be controlled by OBC
									//Put true before activating the statemachine thread. Put false before ending comms thread
bool send_data = false;				//If true, the state machine send packets every airtime
bool send_telemetry = false;		//If true, we have to send telemetry packets instead of payload data
uint8_t num_telemetry = 0;			//Total of telemetry packets that have to be sent (computed when telecomand send telemetry received)
bool contingency = false;			//True if we are in contingency state => only receive

/**
 * Main application entry point.
 */

/*********************************************************************************
* Function: stateMachine														 *
* -----------------------       												 *
*                                                 								 *
* communication process state machine and configure the transceiver and 		 *
* the transmission protocol parameters                                           *
* States:                                                                        *
* -RX_TIMEOUT: when the reception ends                                           *
* -RX_ERROR: when an error in the reception process occurs                       *
* -RX: when a packet has been received                                           *
* -TX: to transmit a packet                                                      *
* -TX_TIMEOUT: when the transmission ends                                        *
* -START_CAD: to detect channel activity (necessary to receive packets correctly)*
* -LOWPOWE: when the transceiver is not transmitting nor receiving               *
*                                                                                *
*  returns: nothing                                                              *
*                                                                                *
**********************************************************************************/
void StateMachine( void )
{
    uint16_t PacketCnt = 0, i=0;
    float Per = 0.0;
    uint16_t bucleCounter = 0;
    uint16_t defaultCounter = 0;
    uint16_t rxCounter = 0;
    uint16_t cadCounter = 0;
    uint16_t failCADCounter = 0;
    uint16_t test_counter = 0;
    uint16_t tx_count = 0;

    uint8_t compare_arrays = 0;

    States_t copy_state = State;	//ERASE AFTER FINISH TESTING
    uint8_t reception_ack_mode = false;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    //Timer used to restart the CAD
    TimerInit( &CADTimeoutTimer, CADTimeoutTimeoutIrq );
    TimerSetValue( &CADTimeoutTimer, CAD_TIMER_TIMEOUT );

    //App timmer used to check the RX's end
    TimerInit( &RxAppTimeoutTimer, RxTimeoutTimerIrq );
    TimerSetValue( &RxAppTimeoutTimer, RX_TIMER_TIMEOUT );

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );

//#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );


    SX126xConfigureCad( CAD_SYMBOL_NUM,CAD_DET_PEAK,CAD_DET_MIN,CAD_TIMEOUT_MS);            // Configure the CAD
    Radio.StartCad( );          // do the config and lunch first CAD

    State = RX;

    //while(  i < NB_TRY )
    while( 1 )
    {
		//DelayMs( 300 );
    	bucleCounter = bucleCounter + 1;

    	/*if (tx_non_stop == 1){
    		DelayMs( 300 );
    	}else{
    		DelayMs( 1 );
    	}*/
    	DelayMs( 1 );
        Radio.IrqProcess( );
        copy_state = State;
        switch( State )
        {
            case RX_TIMEOUT:
            {
                RxTimeoutCnt++;
                //State = START_CAD;
                //Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
				//DelayMs(1);	//Basic RX code
                Radio.Standby;
				State = LOWPOWER;
                break;
            }
            case RX_ERROR:
            {
                RxErrorCnt++;
                PacketReceived = false;
                //State = START_CAD;
                //Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
				//DelayMs(1);	//Basic RX code
                Radio.Standby;
				State = LOWPOWER;
            break;
            }
            case RX:
            {
                if( PacketReceived == true )
                {
					//test_counter = test_counter + 1;
					/*for (uint8_t i=0; i<BUFFER_SIZE; i++){
						MemoryRX[i + rxCounter*BUFFER_SIZE] = Buffer[i];
					}
					rxCounter = rxCounter + 1;
					if (MEMORY_RX_SIZE < ( rxCounter*BUFFER_SIZE + BUFFER_SIZE )){
						rxCounter = 0;
					}*/
					/*if (Buffer[0] == 'T' && Buffer[1] == 'X'){
						//txNonStop = true;
						//txNonStopNum = 1;
						tx_non_stop = 1;
						DelayMs(1000);
					}*/
					if (pin_correct(Buffer[0], Buffer[1])){
						State = LOWPOWER;
						if (Buffer[2] == TLE){
							if (!tle_telecommand){	//First TLE packet
								tle_telecommand = true;
								State = RX;
								telecommand_rx = true;
							}
							else{	//Is the last TLE packet (there are 2)
								tle_telecommand = false;
								State = LOWPOWER;	//Line unnecessary
								telecommand_rx = false;
							}
							process_telecommand(Buffer[2], Buffer[3]);	//Saves the TLE
						}
						else if (telecommand_rx){	//Second telecommand RX consecutively
							rx_attemps_counter = 0;
							if (Buffer[2] == last_telecommand[2]){	//Second telecommand received equal to the first CHANGE THIS TO CHECK THE WHOLE TELECOMMAND. USE VARIABLE compare_arrays
								//Buffer[2] == (SEND_DATA || SEND_TELEMETRY || ACK_DATA || SEND_CALIBRATION || SEND_CONFIG)
								if (Buffer[2] == SEND_DATA || Buffer[2] == SEND_TELEMETRY || Buffer[2] == ACK_DATA || Buffer[2] == SEND_CALIBRATION || Buffer[2] == SEND_CONFIG){
									telecommand_rx = false;
									process_telecommand(Buffer[2], Buffer[3]);
								}
								else {
									request_execution = true;
									State = TX;
									DelayMs(300);
								}
							}
							else if(Buffer[2] == ACK){	//Order execution ACK
								rx_attemps_counter = 0;
								request_counter = 0;
								request_execution = false;
								reception_ack_mode = false;
								telecommand_rx = false;
								process_telecommand(last_telecommand[2], last_telecommand[3]);
								State = RX;
							}
							else{	//Second telecommand received different from the first
							    State = TX;
							    telecommand_rx = false;
							    error_telecommand = true;
							    DelayMs(10);
							}
						}
						else{	//First telecommand RX
							memcpy( last_telecommand, Buffer, BufferSize );
							tle_telecommand = false;
							telecommand_rx = true;
							State = RX;
							rx_attemps_counter = 0;
						}
					} else{	//Pin not correct. If pin not correct it is assumed that the packet comes from another source. The protocol continues ignoring it
					    State = TX;
					    error_telecommand = true;
					    DelayMs(500);
					}
                    PacketReceived = false;     // Reset flag
                }
                else	//If packet not received, restart reception process
                {
                    if (CadRx == CAD_SUCCESS)
                    {
                        channelActivityDetectedCnt++;   // Update counter
                        RxTimeoutTimerIrqFlag = false;
                        TimerReset(&RxAppTimeoutTimer);	// Start the Rx's's Timer
                        //Radio.Rx( RX_TIMEOUT_VALUE );   // CAD is detected, Start RX
                    }
                    else
                    {
                        TimerStart(&CADTimeoutTimer);   // Start the CAD's Timer
                    }
                    Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
                    DelayMs(1);	//Basic RX code
                    State = LOWPOWER;

                	if (reception_ack_mode){
                		reception_ack_mode = false;
                		DelayMs( 300 );
                	}
                }
                break;
            }
            case TX:
            {
            	/* TO TEST TELECOMMANDS */
                State = LOWPOWER;
            	if (error_telecommand){	//Send error message
            		uint8_t paquet_to_send[] = {ERROR,ERROR,ERROR};
            		Radio.Send(paquet_to_send,sizeof(paquet_to_send));
            		DelayMs(100);
            		Radio.Send(paquet_to_send,sizeof(paquet_to_send));	//DISCOMMENT THIS LINE
                    error_telecommand = false;
            	} else if (request_execution){	//Send request for execute telecommand order
            		//paquet_to_send = last_telecommand[2];
            		//Radio.Send(paquet_to_send,1);
            		DelayMs(100);
            		Radio.Send(last_telecommand,3);	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		//Radio.Send(last_telecommand,sizeof(last_telecommand));	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		DelayMs(100);
            		Radio.Send(last_telecommand,3);	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		//Radio.Send(last_telecommand,sizeof(last_telecommand));	//Better here or iterate another time and return to TX?
            		request_counter++;
            		reception_ack_mode = true;
            		State = RX;
            		rx_attemps_counter = 0;
            		//TimerStart(&CADTimeoutTimer);
            		//Radio.Rx( RX_TIMEOUT_VALUE );
            		//PacketReceived = false;
            	} else if (tx_flag || tx_non_stop){	//Send data
                    //txfunction( );
            		uint64_t read_photo[12];
            		uint8_t transformed[96];
            		if (window_paquet < WINDOW_SIZE){
            			Flash_Read_Data(SAVE_PHOTO+paquet_number*96, &read_photo, sizeof(read_photo));
            			Buffer[0] = 0x86;	//Satellite ID
            			Buffer[1] = 0x64;	//Poquetcube ID (there are at least 3)
            			Buffer[2] = paquet_number;	//Number of the paquet
            			memcpy(&transformed, read_photo, sizeof(transformed));
            			for (uint8_t i=3; i<BUFFER_SIZE-1; i++){
            				Buffer[i] = transformed[i-3];
            			}
            			Buffer[BUFFER_SIZE-1] = 0xFF;	//Final of the paquet indicator
            			paquet_number++;
            			window_paquet++;
            			State = TX;
                        DelayMs( 300 );
                        Radio.Send( Buffer, BUFFER_SIZE );
            		} else{
            			tx_non_stop = false;
            			tx_flag = false;
            			send_data = false;
            			window_paquet = 0;
            			State = RX;
            		}
                    DelayMs( 200 );
            	}

            	/* TO TEST PROTOCOL AND SWITCHING BETWEEN STATES
                // Send the next frame
                txfunction();
                //Send Frame
                DelayMs( 1 );
                Radio.Send( Buffer, BUFFER_SIZE );
                for (uint8_t i = 0; i<BUFFER_SIZE; i=i+1){
                	MemoryTX[i+tx_count*BUFFER_SIZE] = Buffer[i];
				}
                tx_count = tx_count + 1;
                if (tx_count*BUFFER_SIZE>MEMORY_RX_SIZE){
                	tx_count = 0;
                }
                */
            	DelayMs( 1 );
                break;
            }
            case TX_TIMEOUT:
            {
                State = LOWPOWER;
                break;
            }
            case LOWPOWER:
            default:
            	defaultCounter = defaultCounter + 1;
                //State = RX;
            	if (tx_non_stop || error_telecommand || tx_flag){
					State = TX;
				}
            	else if (reception_ack_mode || tle_telecommand){
            		State = RX;
            	}
            	else if (telecommand_rx){	//We have received at least one telecommand
            		if (request_execution ){	//In this case we have to TX request or wait for ACK
            			if (request_counter >= 3){	//If 3 request have been sent, we send an error message
            				request_execution = false;
            				error_telecommand = true;
            				telecommand_rx = false;
            				State = TX;
            				request_counter=0;
            				rx_attemps_counter=0;
            			} else if (rx_attemps_counter >= 160){	//TX another request execution
            				State = TX;
            			} else {	//Iterate till 500 ms approx
            				rx_attemps_counter++;
            				State = RX;
            			}
            			/*State = TX;*/// IN THE CASE OF RETRANSMISSIONS OF REQUEST
            			//TimerStart(&CADTimeoutTimer);
            			//Radio.Rx( RX_TIMEOUT_VALUE );
            			//DelayMs(500);
            			//if (!PacketReceived){	//If in 500 ms the RX IRQ does not jump, we go to TX the request or the error
            			//	State = TX;
            			//}
            		} else{	//We want to Rx the second telecommand
            			//TimerStart(&CADTimeoutTimer);
            			//Radio.Rx( RX_TIMEOUT_VALUE );
						//DelayMs(500);
            			rx_attemps_counter++;

            			/*Check the 160 value with the whole code and multithread, because maybe induce a delay*/
            			if (rx_attemps_counter >= 160){	//With this value all 2nd telecommand that arrive before 650 ms are received. 700 ms or more error paquet is send
            				PacketReceived = true;
            				rx_attemps_counter = 0;
            			}
						State = RX; //If Timeout passes and the 2nd telecommand is not received, goes to RX and will process the first, as if the second has been RX
						//PacketReceived = true;
					}
            	}
            	else{
            		State = RX;
            	}

                // Set low power
                break;
        }

        TimerLowPowerHandler( );
        // Process Radio IRQ
        //Radio.IrqProcess( );
    }

}
/*********************************************************************************
* Function: tx_function														     *
* -----------------------                                                        *
* function to transmit the next packet                                           *
*                                                                                *
*  returns: nothing                                                              *
*                                                                                *
**********************************************************************************/
void txfunction( void ){
	uint64_t read_photo[12];
	uint8_t transformed[96];
	if (window_paquet < WINDOW_SIZE){
		Flash_Read_Data(SAVE_PHOTO+paquet_number*96, &read_photo, sizeof(read_photo));
		Buffer[0] = 0x86;	//Satellite ID
		Buffer[1] = 0x64;	//Poquetcube ID (there are at least 3)
		Buffer[2] = paquet_number;	//Number of the paquet
		memcpy(&transformed, read_photo, sizeof(transformed));
		for (uint8_t i=3; i<BUFFER_SIZE-1; i++){
			Buffer[i] = transformed[i-3];
		}
		Buffer[BUFFER_SIZE-1] = 0xFF;	//Final of the paquet indicator
		paquet_number++;
		window_paquet++;
		State = TX;
	} else{
		tx_non_stop = false;
		tx_flag = false;
		send_data = false;
		window_paquet = 0;
		State = RX;
	}
	/*for (uint8_t i = 0; i<BUFFER_SIZE; i=i+1){
		Buffer[i] = Memory[i+txCounter*BUFFER_SIZE];
	}
	txCounter = txCounter + 1;
	if (MEMORY_SIZE < ( txCounter*BUFFER_SIZE + BUFFER_SIZE )){
		txCounter = 0;
		tx_non_stop = 0;
	}*/
}

void OnTxDone( void )
{
    Radio.Standby( );
    if (tx_non_stop == 1){
        State = TX;
    } else{
        State = LOWPOWER;
    }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    uint8_t RXactual[BufferSize];
    memcpy( RXactual, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    PacketReceived = true;
    RssiMoy = (((RssiMoy * RxCorrectCnt) + RssiValue) / (RxCorrectCnt + 1));
    SnrMoy = (((SnrMoy * RxCorrectCnt) + SnrValue) / (RxCorrectCnt + 1));
    State = RX;
    testRX = 1;
}

void OnTxTimeout( void )
{
    Radio.Standby( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Standby( );
    //State = TX;
    //error_telecommand=true;
    State = RX;
    //if (request_execution){
    	//State = TX;
    //}
    if( RxTimeoutTimerIrqFlag )
    {
        State = RX_TIMEOUT;
    }
    else
    {
        Radio.Rx( RX_TIMEOUT_VALUE );   //  Restart Rx
        SymbTimeoutCnt++;               //  if we pass here because of Symbol Timeout
        State = LOWPOWER;
    }
}

void OnRxError( void )
{
    Radio.Standby( );
    State = RX_ERROR;
}

void OnCadDone( bool channelActivityDetected)
{
    Radio.Standby( );

    if( channelActivityDetected == true )
    {
        CadRx = CAD_SUCCESS;
    }
    else
    {
        CadRx = CAD_FAIL;
    }
    State = RX;
}

void SX126xConfigureCad( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout)
{
    SX126xSetDioIrqParams( 	IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                            IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    //SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_ONLY, ((cadTimeout * 1000) / 15.625 ));
    SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, ((cadTimeout * 15.625) / 1000 ));
    //THE TOTAL CAD TIMEOUT CAN BE EQUAL TO RX TIMEOUT (IT SHALL NOT BE HIGHER THAN 4 SECONDS)
}

static void CADTimeoutTimeoutIrq( void )
{
    Radio.Standby( );
    //if (request_execution){
    //	State = TX;
    //} else{
	State = LOWPOWER;
    //}
    //State = START_CAD;
    //State = RX;
}

static void RxTimeoutTimerIrq( void )
{
    RxTimeoutTimerIrqFlag = true;
}

/**************************************************************************************
 *                                                                                    *
 * 	Function:  pin_correct		                                                      *
 * --------------------                                                               *
 * 	check if the pin in the telecommand is correct								      *
 *                                                                                    *
 *  pin_1: first byte of the pin			                                          *
 *  pin_2: second byte of the pin													  *
 *                                                                                    *
 *  returns: true if correct							                              *
 *                                                                                    *
 **************************************************************************************/
bool pin_correct(uint8_t pin_1, uint8_t pin_2) {
	if (pin_1 == PIN1 && pin_2 == PIN2){
		return true;
	}
	return false;
}


/**************************************************************************************
 *                                                                                    *
 * 	Function:  process_telecommand                                                    *
 * --------------------                                                               *
 * 	processes the information contained in the packet depending on the telecommand    *
 * 	received																	      *
 *                                                                                    *
 *  header: number of telecommand			                                          *
 *  info: information contained in the received packet								  *
 *                                                                                    *
 *  returns: nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void process_telecommand(uint8_t header, uint8_t info) {
	uint64_t info_write;
	switch(header) {
	case RESET2:
		HAL_NVIC_SystemReset();  //resets the whole PQ
		break;
	case NOMINAL:  //Modifies the nominal threshold used to change between each state.
		info_write = info;
		Flash_Write_Data(NOMINAL_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", NOMINAL_NOTIFICATION); //Notification to OBC
		break;
	case LOW: //Modifies the low threshold used to change between each state.
		info_write = info;
		Flash_Write_Data(LOW_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", LOW_NOTIFICATION); //Notification to OBC
		break;
	case CRITICAL: //Modifies the critical threshold used to change between each state.
		info_write = info;
		Flash_Write_Data(CRITICAL_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", CRITICAL_NOTIFICATION"); //Notification to OBC
		break;
	case EXIT_CONTINGENCY:{  //Leave the low power state
		info_write = info;
		Flash_Write_Data(EXIT_LOW_POWER_FLAG_ADDR, &info_write, 1);
		info_write=1;
		Flash_Write_Data(EXIT_LOW_ADDR, &info_write, 1); //info_write has a value of 1 because of its TRUE to exit this state
		//xTaskNotify("Task OBC", EXITLOWPOWER_NOTIFICATION); //Notification to OBC
		break;
	}
	case EXIT_SUNSAFE:{ //Leave the sunsafe state
		Flash_Write_Data(EXIT_LOW_POWER_FLAG_ADDR, &info, 1);
		info_write=1;
		Flash_Write_Data(EXIT_SUNSAFE, &info_write, 1); //info_write has a value of 1 because of its TRUE to exit this state
		//xTaskNotify("Task OBC", EXITSUNSAFE_NOTIFICATION); //Notification to OBC
		break;
	}
	case SET_TIME:{ //Synchronises the time of both the satellite and the ground station
		uint8_t time[4];  //is stored in a 4byte vector
		for (k=0; k<4; k++){
			time[k]=Buffer[k+1];  //stored in the buffer
		}
		Flash_Write_Data(TEMP_ADDR, &time, sizeof(time)); //unix time format
		//xTaskNotify("Task OBC", SETTIME_NOTIFICATION);
		//uint8_t time[4];  //4bytes
		//	uint8_t time[TIME_SIZE];
					/*for(n=0; n<4; n++){
						time[n]=Buffer[n+1];
					}
					Write_Flash(TEMP_ADDR, time, sizeof(time));
					//todo: posarho dintre un bucle
					//xTaskNotify("Task OBC", SETTIME_NOTIFICATION, eSetBits);
					//if(xTaskNotifyWait(0x00, ULONG_MAX, &NotifiedValue, portMAXVALUE) == pdTRUE){    //esperar x l'OBC (TIMERTC_NOTI)
					//			// llegir l'adreça TIMERTC_ADDR
								//Read_Flash(TIMERTC_ADDR, &time, TIME_SIZE);
								//enviar el que hi ha a l'adreça a la GS
								//Radio.Send(time, TIME_SIZE);
					//}*/

		break;
	}
	case SET_CONSTANT_KP:    //Updates the current constant proportional Kp
		info_write=info;
		Flash_Write_Data(KP_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", SETCONSTANT_NOTIFICATION); //Notification to OBC
		break;
	case TLE:{  //Updates the current TLE of the orbit. Done in the ground station since its easier than doing it in the satellite.
				//TLE’s are made with 2 arrays of 69 characters 138 bytes
		/*
		uint8_t tle[TLE_PACKET_SIZE];
		for (k=0; k<TLE_PACKET_SIZE; k++){
			tle[k]=Buffer[k+3];
		}
		Flash_Write_Data(TEST_ADDRESS2 + tle_packets*(TLE_PACKET_SIZE), &tle, sizeof(tle));
		if (tle_packets > 0){
			tle_packets = 0;
		}else{
			tle_packets++;
		}*/
		//SEE WHY THE PREVIOUS CODE DOES NOT WORKS
		if (tle_packets == 0){
			uint8_t tle[TLE_PACKET_SIZE];
			for (k=0; k<TLE_PACKET_SIZE; k++){
				tle[k]=Buffer[k+3];
			}
			Flash_Write_Data(TEST_ADDRESS2 + tle_packets*(TLE_PACKET_SIZE), &tle, sizeof(tle));
			tle_packets++;
		} else{
			uint8_t tle2[TLE_PACKET_SIZE];
			for (k=0; k<TLE_PACKET_SIZE; k++){
				tle2[k]=Buffer[k+3];
			}
			Flash_Write_Data(TEST_ADDRESS2 + tle_packets*(TLE_PACKET_SIZE), &tle2, sizeof(tle2));
			tle_packets++;
			tle_packets = 0;
		}
		//xTaskNotify("Task OBC", TLE_NOTIFICATION) //Notification to OBC
		break;
	}
	case SET_GYRO_RES: //Modifies the resolution needed for each gyroscope measurement.The starting value will be 16 (16 bits per measurement)
		/*4 possibles estats, rebrem 00/01/10/11*/
		info_write=info;
		Flash_Write_Data(GYRO_RES_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", SETGYRO_NOTIFICATION)//Notification to OBC
		break;
	case SEND_DATA:{ //Informs the satellite when transmission is available
		//Send data from payload. In the case of payload 1,there is a parameter to request big or small photos
		if (!contingency){ //if we are not in contingency we enter
			tx_flag = true;	//Activates TX flag
			State = TX;
			send_data = true;
		}
		break;
	}
	case SEND_TELEMETRY:{ //Tells the satellite to send all the telemetry data acquired
		/* SEND TELEMETRY DIRECTLY FROM HERE WITH RADIO.SEND */
		if (!contingency){
			send_telemetry = true;
			num_telemetry = (uint8_t) 34/BUFFER_SIZE + 1; //cast to integer to erase the decimal part
			//State = TX;
		}
		break;
	}
	case STOP_SENDING_DATA:{ //Tell the transceiver to stop sending data
		send_data = false;
		count_packet[0] = 0; //puts the counter to 0 because of we are saying to stop
		break;
	}
	case ACK_DATA:{ //ACK packet to acknowledge when a packet is received.
		//check it
	 	 ack = ack & Buffer[1];
		 for(j=2; j<ACK_PAYLOAD_LENGTH; j++){
			 ack = (ack << 8*j) & Buffer[j];
		 }
		 count_window[0] = 0;
		 full_window = false;
		 if (ack != 0xFFFFFFFFFFFFFFFF){
			 nack = true;
		 }
		 //State = TX;
		break;
	}
	case SET_SF_CR: { //Allows the ground station to modify the SF of the modulation whenever possible. At higher elevation angles, the SF can be lowered, thus increasing the average bitrate.
		uint8_t SF;
		if (info == 0) SF = 7;
		else if (info == 1) SF = 8;
		else if (info == 2) SF = 9;
		else if (info == 3) SF = 10;
		else if (info == 4) SF = 11;
		else if (info == 5) SF = 12;
		info_write=SF;
		Flash_Write_Data(SF_ADDR, &info_write, 1);
		/*4 cases (4/5, 4/6, 4/7,1/2), so we will receive and store 0, 1, 2 or 3*/
		Flash_Write_Data(CRC_ADDR, &Buffer[2], 1);
		break;
	}
	case SEND_CALIBRATION:{	//Rx calibration //Sends all the parameters needed for calibration purposes
		/* RX OR SEND???? */
		/* IT TX, IT CAN BE DONE WITH ONE PACKET DIRECTLY FROM HERE RADIO.SEND */
		uint8_t calib[UPLINK_BUFFER_SIZE-1];
		for (k=1; k<UPLINK_BUFFER_SIZE; k++){
			calib[k-1]=Buffer[k];
		}
		Flash_Write_Data(CALIBRATION_ADDR, &calib, sizeof(calib));
		calib_packets = calib_packets + 1;
		uint8_t integer_part = (uint8_t) 138/UPLINK_BUFFER_SIZE;
		if(calib_packets == integer_part+1){
			calib_packets = 0;
		}
		//xTaskNotify("Task OBC", SENDCALIBRATION_NOTIFICATION); //Notification to OBC
		break;
	}
	case TAKE_PHOTO:{ //Informs the satellite on when totake a photo, Sets the photo resolution, Sets the photo compression
		/*GUARDAR TEMPS FOTO?*/
		info_write=1;
		Flash_Write_Data(PAYLOAD_STATE_ADDR, &info_write, 1); //info_write has a value of 1 because of its TRUE to exit this state
		Flash_Write_Data(PL_TIME_ADDR, &info, 4);
		Flash_Write_Data(PHOTO_RESOL_ADDR, &Buffer[5], 1);
		Flash_Write_Data(PHOTO_COMPRESSION_ADDR, &Buffer[6], 1);
		//xTaskNotify("Task OBC", TAKEPHOTO_NOTIFICATION); //Notification to OBC
		break;
	}
	case TAKE_RF:{
		info_write=1;
		Flash_Write_Data(PAYLOAD_STATE_ADDR, &info_write, 1); //info_write has a value of 1 because of its TRUE to exit this state, Informs the satellite on when to start and finish taking RF measurements with the electrosmog antenna
		Flash_Write_Data(PL_TIME_ADDR, &info, 8); //Sets the integration time needed for the electrosmog antenna measurements
		Flash_Write_Data(F_MIN_ADDR, &Buffer[9], 1); //Sets the minimum frequency that will be swept by the electrosmog antenna.
		Flash_Write_Data(F_MAX_ADDR, &Buffer[10], 1); //Sets the maximum frequency that will be swept by the electrosmog antenna.
		Flash_Write_Data(DELTA_F_ADDR, &Buffer[11], 1); //Sets the step length (resolution) between each measurement
		Flash_Write_Data(INTEGRATION_TIME_ADDR, &Buffer[12], 1);
		//xTaskNotify("Task OBC", TAKERF_NOTIFICATION); //Notification to OBC
		//POSSIBILITAT D'INCLOURE, LUT amb llistat de freq ????Adriano ho va comentar
		break;
	}
	case SEND_CONFIG:{ //Tells the satellite to send all the current configurations to the ground station to check if everything is correctly configured.
		uint8_t config[CONFIG_SIZE];
		info_write=config;
		Flash_Read_Data(CONFIG_ADDR, &info_write, CONFIG_SIZE);
		Radio.Send( config, CONFIG_SIZE );
		break;
	}
	default:{ //this is the default case
		State = TX;
		error_telecommand = true;
	}
	}
}
//FUNCTION TO TEST ONLY THE FUNCTION: process_telecommands()
//void telecommands(void){
//
//		uint8_t prova =0;
//		uint8_t info=3;
//		for(prova=11; prova<50 ; prova++){
//			process_telecommand(prova, info);
//			info=info+3;
//		}
//
//}

//void beacon_timmer_irq(void){
// 	if(txflag==true){
//			Tx_beacon=false;
//			State=TX;
//	}
//	else{
//		if(tleflag==true){
//			Tx_beacon=false;
//			State=RX;
//		}
//		else{
//			Tx_beacon=true;
//			State=TX;
//		}
//	}
//}

