/**
  ******************************************************************************
  * @file    
  * @author  
  * @brief  
  *
  *          This file contains:
  *           - 
  *           - 
  *           - 
  *
  ******************************************************************************
**/
/**********Includes************************************************************/
#include "Farand_Ethernet.h"
#include "main.h"

/**********Variables Definition************************************************/
uint8_t cable_Connection_State = CONNECTED;
uint8_t change_state = 1;
uint32_t network_Timeout_Counter = 0;
uint16_t TaskMemory_send;
uint16_t TaskMemory_rcv;
uint8_t send_Buffer_Flag = IDLE;
uint8_t send_buf[512];

struct Client uC_F7_Client;
/**********Function Prototypes************************************************/
uint8_t Write_Buffer_On_Ethernet(struct netconn *conn,uint8_t* Buf, uint32_t Len);
void Network_Timeout(void const * argument);
void TCP_Client_Receive_Task(void const * argument);
void Send_Via_Ethernet(uint8_t* buf, uint16_t len);


Recv_Struct Recieve_From_Ethernet(struct netconn *conn);
/**********Function Definitions************************************************/
void TCP_Client_Send_Task(void const * argument)
{		
	/* Microcontroller is client for Computer or GPU*/
	
	//printf("client task started\n");	
	uC_F7_Client.Network_Status = DISCONNECTED;

	while(1)
	{		
		
		HAL_GPIO_TogglePin(Test_con603_pin3_GPIO_Port,Test_con603_pin3_Pin);// test toggle pin
		
		if(cable_Connection_State == CONNECTED && uC_F7_Client.Network_Status == DISCONNECTED)
		{			
			// Set Server IP Adderss (Computer or GPU)
			IP4_ADDR(&uC_F7_Client.ServerIPAddr, SERVERIP_0, SERVERIP_1, SERVERIP_2, SERVERIP_3);
			uC_F7_Client.conn = netconn_new(NETCONN_TCP);
			
			// Try Connecting to Serever
 			uC_F7_Client.connection_err = netconn_connect(uC_F7_Client.conn, &uC_F7_Client.ServerIPAddr, SERVER_Port); 			
			if( uC_F7_Client.connection_err != ERR_OK)
			{				
				netconn_delete(uC_F7_Client.conn);
			}
			else 
			{
				uC_F7_Client.Network_Status = CONNECTED;
				uC_F7_Client.Retry_Count = 0;
				netconn_set_recvtimeout(uC_F7_Client.conn,100); //time out is in milisecond
			}			
			
			// If retry count exceeds maximum, Terminate the Client Task			
			uC_F7_Client.Retry_Count++;		
			if(uC_F7_Client.Retry_Count > SERVER_CONNECTION_TRY_COUNT_MAX)
			{
				uC_F7_Client.Retry_Count = 0;				
				netconn_delete(uC_F7_Client.conn);	
				
				if(xTimerIsTimerActive(NetworkTimeoutTimerHandle) == 0)	
				{
					osTimerStart(NetworkTimeoutTimerHandle, NETWORK_RECONNECT_TIMEOUT);
				}
		
				osThreadTerminate(tcpClinetTaskHandle);		
			}			
		}
		else if(cable_Connection_State == DISCONNECTED)
		{
			// Toggle RED LED if Ethernet cable is disconnected
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Send to Server
		if (uC_F7_Client.Network_Status == CONNECTED )
		{	
			
			// Send 
//			if( send_Buffer_Flag == READY)
//			{
				send_Buffer_Flag = IDLE;
				uC_F7_Client.send_err = Write_Buffer_On_Ethernet(uC_F7_Client.conn, ethernet_Tx_Bytes, 512);
			
				// if Sending is not successful, delete the connection
				if( uC_F7_Client.send_err != ERR_OK ) 
				{
					uC_F7_Client.Network_Status = DISCONNECTED;		
					netconn_delete(uC_F7_Client.conn);
				}
			//}
		}
		
		if (uC_F7_Client.Network_Status == CONNECTED)
		{
			
		}
		TaskMemory_send =  uxTaskGetStackHighWaterMark2(NULL);
		
		osDelay(20);
	}
}
uint8_t Write_Buffer_On_Ethernet(struct netconn *conn,uint8_t* Buf, uint32_t Len)
{
	err_t err;
	err = netconn_write(conn, Buf, Len, NETCONN_COPY);
	//err = netconn_write_partly(conn,Buf,Len,NETCONN_COPY,NULL);
	if( err == ERR_OK)
	{
		return 0;
	}	
	else return 1;
}
Recv_Struct Recieve_From_Ethernet(struct netconn *conn)
{
	uint8_t* buf;
	uint16_t len;
	Recv_Struct recv;
	struct netbuf *recvbuf;
	recv.recv_err = netconn_recv(conn, &recvbuf);
	if(recv.recv_err == ERR_OK)
	{
		netbuf_data(recvbuf, (void **)&buf, &len);
		recv.dataptr = buf;
		recv.Length_of_Array = len;
		netbuf_delete(recvbuf);		
	}
	
	return recv;	
}



void TCP_Client_Receive_Task(void const * argument)
{
	//Receive data from Server as a Client
	uint32_t Data_length;	
	Recv_Struct Client_recv_struct;
	for(;;)
	{		
		if(uC_F7_Client.Network_Status == CONNECTED)
		{			
			Client_recv_struct = Recieve_From_Ethernet(uC_F7_Client.conn);
			
			if(Client_recv_struct.recv_err == 0 )
			{
				
				Data_length = Client_recv_struct.Length_of_Array;
				for(uint32_t i = 0 ; i < Data_length ; i++)
				{

					ethernet_Rx_Bytes[i] = Client_recv_struct.dataptr[i];
				}
				//for test
//				Report_to_PC();
	
	      
				Service_Input_Command(ethernet_Rx_Bytes);				
			}										
		}
		//TaskMemory_rcv =  uxTaskGetStackHighWaterMark2(NULL);
		osDelay(10);
	}
}


void Network_Timeout(void const * argument)
{
	if(uC_F7_Client.Network_Status == DISCONNECTED)
	{
		network_Timeout_Counter++;
	//	HAL_GPIO_TogglePin(LED_BLU_GPIO_Port,LED_BLU_Pin);
	}	
	
	if( network_Timeout_Counter == 5 * RETRY_INTERVAL && uC_F7_Client.Network_Status == DISCONNECTED) 
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET);		
		network_Timeout_Counter = 0;
		
		osThreadDef(tcpclientTask, TCP_Client_Send_Task, osPriorityNormal, 0, 500);
		tcpClinetTaskHandle = osThreadCreate(osThread(tcpclientTask), NULL);		
	}		
	else 
	{
		if(xTimerIsTimerActive(NetworkTimeoutTimerHandle) == 0)	
		{
			osTimerStart( NetworkTimeoutTimerHandle, NETWORK_RECONNECT_TIMEOUT );
		}
	}
  /* USER CODE END Callback01 */
}



void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file 
            when the callback is needed */
	change_state = 1;

	if( cable_Connection_State == 1 && change_state == 1)		 
	{
		cable_Connection_State = 0;
		change_state = 0;
		
	}
	if( cable_Connection_State == 0 && change_state == 1) 
	{
		cable_Connection_State = 1;
		change_state = 0;		
		netif_set_up(&gnetif);
//		osTimerStop(NetworkTimeoutTimerHandle);
	}	
}

void Send_Via_Ethernet(uint8_t* buf, uint16_t len)
{
	send_Buffer_Flag = BUSY;
	for(int i=0; i < len; i++)
	{
		ethernet_Tx_Bytes[i] = buf[i];
	}
	send_Buffer_Flag = READY;
}


