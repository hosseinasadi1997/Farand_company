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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __farand_ethernet_H
#define __farand_ethernet_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "api.h"
#include "netif.h"

#define TCP TRUE
#define CLIENT TRUE

// Server IP Address (Computer or GPU)
#define SERVER_Port 5050
#define SERVERIP_0 192
#define SERVERIP_1 168
#define SERVERIP_2 1
#define SERVERIP_3 70

#define CONNECTED 												1
#define DISCONNECTED  										0
#define NETWORK_RECONNECT_TIMEOUT 					100//20000 //set this number in miliseconds
#define SERVER_CONNECTION_TRY_COUNT_MAX			20
#define RETRY_INTERVAL 									4//5000

#define BUSY  0
#define READY 1
#define IDLE  2
/**********Includes*********************************************************/

//This Header Contains All The Functions Prototypes For The HAL Module Driver

/**********External Variabes************************************************/
extern void TCP_Client_Send_Task(void const * argument);
extern void TCP_Client_Receive_Task(void const * argument);
extern void Network_Timeout(void const * argument);
extern uint8_t Write_Buffer_On_Ethernet(struct netconn *conn,uint8_t* Buf, uint32_t Len);
extern void Send_Via_Ethernet(uint8_t* buf, uint16_t len);


/**********Typedefs*********************************************************/

extern osThreadId tcpClinetTaskHandle;
extern osThreadId tcpClientRecvCallbackHandle;
extern osTimerId NetworkTimeoutTimerHandle;

extern struct Client uC_F7_Client;
extern struct netif gnetif;

struct Client 
{
	struct netconn *conn;
	ip_addr_t ServerIPAddr;
	struct netbuf *recvbuf;
	uint8_t Retry_Count;
	uint8_t Network_Status;
	err_t connection_err;
	err_t send_err;
	err_t recv_err;
};

typedef struct
{
	struct netconn *server_conn, *client_conn;
	struct netbuf *recvbuf;
	err_t bind_err;
	err_t accept_err;
	err_t send_err;
	err_t recv_err;
	uint8_t accept_state;
}TCP_server_Struct;

struct Received_Data_From_Ethernet { 
		err_t recv_err;
    uint8_t *dataptr;
		uint32_t Length_of_Array;
}; 

typedef struct Received_Data_From_Ethernet Recv_Struct;


#ifdef __cplusplus
}
#endif
#endif
/**********END OF FILE******************************************************/
