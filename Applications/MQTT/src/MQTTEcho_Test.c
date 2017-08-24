/*
 * MQTTEcho_Test.c
 *
 *  Created on: 10.08.2017
 *      Author: CHristoph.Domnik
 *
 *  This program is only used to try out the MQTT functionalitie
 *  and will not beimplementet in the later builds
 */


/* Standard includes. */
#include <stdint.h>
#include <stdlib.h>

#include "MQTTClient.h"
#include "MQTTEcho_Test.h"
#include "MQTTFreeRTOS.h"


void messageArrived(MessageData* data)
{
	printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
		data->message->payloadlen, data->message->payload);
}


static void prvMQTTEchoTask(void *pvParameters)
{
	vTaskDelay(10000); // Wait 10s to don't interrupt the IP Stack StartUp

	/* connect to m2m.eclipse.org, subscribe to a topic, send and receive messages regularly every 1 sec */
	MQTTClient client;
	Network network;
	unsigned char sendbuf[100], readbuf[10];
	int rc = 0,
		count = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

	pvParameters = 0;
	NetworkInit(&network);
	MQTTClientInit(&client, &network, 20000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));
	client.thread.task = NULL;

	char* address = MQTT_SERVER;
	//char* address = "iot.eclipse.org";
	rc = NetworkConnect( &network, address, 1883 );
	printf("Return code from network connect is %d\n", rc);
	if ( rc != 0 )
		goto exit;

	if ((rc = MQTTStartTask(&client)) != pdPASS)
		printf("Return code from start tasks is %d\n", rc);

	connectData.MQTTVersion = 4;
	connectData.clientID.cstring = MQTT_CLIENTID;
	connectData.keepAliveInterval = 180;
	connectData.cleansession = 1;
	connectData.willFlag = 0;
	connectData.username.cstring = MQTT_USER;
	connectData.password.cstring = MQTT_PASSWORD;

	rc = MQTTConnect(&client, &connectData);
	printf("Return code from MQTT connect is %d\n", rc);
	if ( rc != 0 ){
		MQTTDisconnect( &client );
		goto exit;
	}
	else
		printf("MQTT Connected\n");

	while (++count)
	{
		MQTTMessage message;
		char payload[30];

		message.qos = 0;
		message.retained = 0;
		message.payload = payload;
		sprintf(payload, "{\"meaning\":\"TestValue\", \"value\":%d}", count);
		message.payloadlen = strlen(payload);

		rc = MQTTPublish(&client, MQTT_TOPIC, &message);
		printf("Return code from MQTT publish is %d\n", rc);
		if ( rc != 0 ) {
			MQTTDisconnect( &client );
			goto exit;
		}

		vTaskDelay(1000);
	}

exit:
	FreeRTOS_shutdown( network.my_socket, FREERTOS_SHUT_RDWR );

	rc = 0;
	while( FreeRTOS_recv( network.my_socket, readbuf, 0, 0 ) >= 0 || ( rc == 20 ) )
    {
        vTaskDelay( 250 );
        rc++;
    }

    /* The socket has shut down and is safe to close. */
    FreeRTOS_closesocket( network.my_socket );


	printf("could not connect\n");

	if( client.thread.task != NULL )
		vTaskDelete( client.thread.task );

	vTaskDelete( NULL );
}


void vStartMQTTTasks(uint16_t usTaskStackSize, UBaseType_t uxTaskPriority)
{
	BaseType_t x = 0L;

	xTaskCreate(prvMQTTEchoTask,	/* The function that implements the task. */
			"MQTTEcho0",			/* Just a text name for the task to aid debugging. */
			usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
			(void *)x,		/* The task parameter, not used in this case. */
			uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
			NULL);				/* The task handle is not used. */
}
/*-----------------------------------------------------------*/

void vTestTask()
{
	vTaskDelay(10000);

	int8_t cBuffer[ 16 ];
	Socket_t xTCPClientSocket;
	struct freertos_sockaddr sAddr;
	int retVal;

	while(1)
	{
		//Create Socket for TCP Client
		xTCPClientSocket = FreeRTOS_socket( FREERTOS_AF_INET,
		    								FREERTOS_SOCK_STREAM,
											FREERTOS_IPPROTO_TCP);

		if ( xTCPClientSocket != FREERTOS_INVALID_SOCKET )
		{
			printf("TCP Socket created.\n");
			retVal = FreeRTOS_bind( xTCPClientSocket, NULL, sizeof( struct freertos_sockaddr )  );
			if ( retVal == 0 )
			{
				sAddr.sin_port = FreeRTOS_htons( 80 );
				sAddr.sin_addr = 1845602496;

				/* Convert the IP address to a string. */
				FreeRTOS_inet_ntoa( sAddr.sin_addr, ( char * ) cBuffer );

				/* Print out the IP address. */
				printf( "DVT Server is at IP address %s\r\n", cBuffer );

				retVal = FreeRTOS_connect( xTCPClientSocket, &sAddr, sizeof( sAddr ) );
				if( retVal == 0 )
				{
					printf("Finished Test Successfully.\n");
		    	}
				else
					printf("Error, unable to connect to DVT Server\n");
		    }
			else
				printf("Error, unable to bind TCP Socket\n");
		}
		else
			printf("Error creating TCP Socket\n");

		FreeRTOS_closesocket( xTCPClientSocket );
		vTaskDelete( NULL );
	}
}


void vLookUpAddress()
{
	xTaskCreate(vTestTask,
			"MQTTTest",
			240,
			NULL,
			3,
			NULL);
}
