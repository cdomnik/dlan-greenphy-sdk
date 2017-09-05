/*
 * Expand2Click.c
 *
 *  Created on: 29.12.2016
 *      Author: devolo AG / mikroelektronika
 */

/* Standard includes. */
#include <string.h>
#include <stdlib.h>

/* LPCOpen Includes. */
#include "board.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* GreenPHY SDK includes. */
#include "GreenPhySDKConfig.h"
#include "GreenPhySDKNetConfig.h"
#include "http_query_parser.h"
#include "http_request.h"
#include "clickboard_config.h"
#include "expand2click.h"

#if( netconfigUSEMQTT != 0 )
	/* MQTT includes */
	#include "mqtt.h"
#endif

/* Task-Delay in ms, change to your preference */
#define TASKWAIT_EXPAND2 100

/*****************************************************************************/

#define EXPAND_ADDR (0x00)   /*jumper A0;A1;A2 on expand2click board*/

//sbit EXPAND_RST at GPIOC_ODR.B2;

// unsigned char i = 0, old_res = 0, res;

//extern sfr sbit EXPAND_RST;

char Expander_Read_Byte(char ModuleAddress, char RegAddress){
  I2C_XFER_T xfer;
  char temp;
  xfer.slaveAddr = AddressCode|ModuleAddress;
  uint8_t tmp_data[2];

  tmp_data[0] = RegAddress;
  xfer.txBuff = tmp_data;
  xfer.rxBuff = tmp_data;
  xfer.txSz = 1;
  xfer.rxSz = 1;

  Chip_I2C_MasterSend(I2C1,xfer.slaveAddr,  xfer.txBuff, xfer.txSz);
  Chip_I2C_MasterRead(I2C1,xfer.slaveAddr,  xfer.rxBuff, xfer.rxSz);

  temp   = tmp_data[0];
  return temp;
}

void Expander_Write_Byte(char ModuleAddress,char RegAddress, char Data_) {
	  I2C_XFER_T xfer;
	  xfer.slaveAddr = AddressCode|ModuleAddress;
	  uint8_t tmp_data[2];
	  tmp_data[0] = RegAddress;
	  tmp_data[1] = Data_;
	  xfer.txBuff = tmp_data;
	  //xfer.rxBuff = tmp_data;
	  xfer.txSz = 2;
	  //xfer.rxSz = 2;
	  Chip_I2C_MasterSend(I2C1,xfer.slaveAddr, xfer.txBuff, xfer.txSz);
}

/*****************************************************************************/

/* Task handle used to identify the clickboard's task and check if the
clickboard is activated. */
static TaskHandle_t xClickTaskHandle = NULL;

/* Output bits managed by the Expand2Click task.
If a bit is set to 1 the pin is low. */
static char oBits = 0;

/* Input bits managed by the Expand2Click task.
If a bit is set to 1 the pin is low. */
static char iBits = 0;

/* Configurate on witch pins the water meter is connected */
static char togglePins[2] = { 0, 0 };

/* Count how often the input bits were toggled. */
static int toggleCount[2] = { 0, 0 };

static int multiplicator = 250;

static unsigned char newVal = 0;

/*-----------------------------------------------------------*/

static char get_expand2click(void){
	return (Expander_Read_Byte(EXPAND_ADDR, GPIOA_BANK0));  // Read expander's PORTA
}
/*-----------------------------------------------------------*/

void set_expand2click(char pins){
	Expander_Write_Byte(EXPAND_ADDR, OLATB_BANK0, pins);    // Write pins to expander's PORTB
}
/*-----------------------------------------------------------*/
BaseType_t xExpand2Click_Deinit ( void );

static void vClickTask(void *pvParameters)
{
const TickType_t xDelay = TASKWAIT_EXPAND2 / portTICK_PERIOD_MS;
char lastBits = get_expand2click();
//char count = 0;
#if( netconfigUSEMQTT != 0 )
	char buffer[40];
#endif /* #if( netconfigUSEMQTT != 0 ) */

	while (1) {
		/* Toggle obits once per second - just for demo. */
//		if( count++ % 10 == 0 )
//			oBits ^= ( togglePins[0] | togglePins[1] );

		set_expand2click(oBits);

		/* Get iBits from board */
		iBits = get_expand2click();

		/* First water meter pin toggled? */
		if( ( iBits ^ lastBits ) & togglePins[0] ){
			toggleCount[0] += 1;
			#if( netconfigUSEMQTT != 0 )
				sprintf(buffer, "{\"meaning\":\"wmeter1\",\"value\":%d}", toggleCount[0]);
				xPublishMessage( buffer, netconfigMQTT_TOPIC, 0, 0 );
			#endif /* #if( netconfigUSEMQTT != 0 ) */
		}

		/* Second water meter pin toggled? */
		if( ( iBits ^ lastBits ) & togglePins[1] ){
			toggleCount[1] += 1;
			#if( netconfigUSEMQTT != 0 )
				sprintf(buffer, "{\"meaning\":\"wmeter2\",\"value\":%d}", toggleCount[1]);
				xPublishMessage( buffer, netconfigMQTT_TOPIC, 0, 0 );
			#endif /* #if( netconfigUSEMQTT != 0 ) */
		}

		lastBits = iBits;

		vTaskDelay( xDelay );
	}
}
/*-----------------------------------------------------------*/

#if( includeHTTP_DEMO != 0 )
	static BaseType_t xClickHTTPRequestHandler( char *pcBuffer, size_t uxBufferLength, QueryParam_t *pxParams, BaseType_t xParamCount )
	{
		BaseType_t xCount = 0;
		QueryParam_t *pxParam;

		// Search object for 'output' Parameter to get the new Value of oBits
		pxParam = pxFindKeyInQueryParams( "output", pxParams, xParamCount );
		if( pxParam != NULL ) {
			oBits = strtol( pxParam->pcValue, NULL, 10 );
		}

		pxParam = pxFindKeyInQueryParams( "pin0", pxParams, xParamCount );
		if( pxParam != NULL ) {
			toggleCount[0] = 0;
			togglePins[0] = strtol( pxParam->pcValue, NULL, 10 );
		}

		pxParam = pxFindKeyInQueryParams( "pin1", pxParams, xParamCount );
		if( pxParam != NULL ) {
			toggleCount[1] = 0;
			togglePins[1] = strtol( pxParam->pcValue, NULL, 10 );
		}

		pxParam = pxFindKeyInQueryParams( "multi", pxParams, xParamCount );
		if( pxParam != NULL ) {
			multiplicator = strtol( pxParam->pcValue, NULL, 10 );
		}

		xCount += sprintf( pcBuffer, "{"
				"\"input\":"  "%d,"
				"\"output\":" "%d,"
				"\"count0\":" "%d,"
				"\"count1\":" "%d,"
				"\"pin0\":"   "%d,"
				"\"pin1\":"   "%d,"
				"\"multi\":"  "%d"
			"}",
			iBits,
			oBits,
			toggleCount[0],
			toggleCount[1],
			togglePins[0],
			togglePins[1],
			multiplicator
		);
		return xCount;
	}
#endif
/*-----------------------------------------------------------*/

BaseType_t xExpand2Click_Init ( const char *pcName, BaseType_t xPort )
{
BaseType_t xReturn = pdFALSE;

	/* Use the task handle to guard against multiple initialization. */
	if( xClickTaskHandle == NULL )
	{
		/* Configure GPIOs depending on the microbus port. */
		if( xPort == eClickboardPort1 )
		{
			/* Set interrupt pin. */
			Chip_GPIO_SetPinDIRInput(LPC_GPIO, CLICKBOARD1_INT_GPIO_PORT_NUM, CLICKBOARD1_INT_GPIO_BIT_NUM);
			/* Set reset pin. */
			Chip_GPIO_SetPinDIROutput(LPC_GPIO, CLICKBOARD1_RST_GPIO_PORT_NUM, CLICKBOARD1_RST_GPIO_BIT_NUM);
			/* Start  reset procedure. */
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, CLICKBOARD1_RST_GPIO_PORT_NUM, CLICKBOARD1_RST_GPIO_BIT_NUM);
			Chip_GPIO_SetPinOutLow( LPC_GPIO, CLICKBOARD1_RST_GPIO_PORT_NUM, CLICKBOARD1_RST_GPIO_BIT_NUM);
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, CLICKBOARD1_RST_GPIO_PORT_NUM, CLICKBOARD1_RST_GPIO_BIT_NUM);
		}
		else if( xPort == eClickboardPort2 )
		{
			/* Set interrupt pin. */
			Chip_GPIO_SetPinDIRInput(LPC_GPIO, CLICKBOARD2_INT_GPIO_PORT_NUM, CLICKBOARD2_INT_GPIO_BIT_NUM);
			/* Set reset pin. */
			Chip_GPIO_SetPinDIROutput(LPC_GPIO, CLICKBOARD2_RST_GPIO_PORT_NUM, CLICKBOARD2_RST_GPIO_BIT_NUM);
			/* Start  reset procedure. */
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, CLICKBOARD2_RST_GPIO_PORT_NUM, CLICKBOARD2_RST_GPIO_BIT_NUM);
			Chip_GPIO_SetPinOutLow( LPC_GPIO, CLICKBOARD2_RST_GPIO_PORT_NUM, CLICKBOARD2_RST_GPIO_BIT_NUM);
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, CLICKBOARD2_RST_GPIO_PORT_NUM, CLICKBOARD2_RST_GPIO_BIT_NUM);
		}

		/* Initialize I2C. Both microbus ports are connected to the same I2C bus. */
		Board_I2C_Init( I2C1 );

		/* Initialize Expand2Click chip. */
		Expander_Write_Byte(EXPAND_ADDR, IODIRB_BANK0, 0x00);  // Set Expander's PORTB to be output
		Expander_Write_Byte(EXPAND_ADDR, IODIRA_BANK0, 0xFF);  // Set Expander's PORTA to be input
		Expander_Write_Byte(EXPAND_ADDR, GPPUA_BANK0, 0xFF);   // Set pull-ups to all of the Expander's PORTA pins

		/* Create task. */
		xTaskCreate( vClickTask, pcName, 240, NULL, ( tskIDLE_PRIORITY + 1 ), &xClickTaskHandle );
		if( xClickTaskHandle != NULL )
		{
			#if( includeHTTP_DEMO != 0 )
			{
				/* Add HTTP request handler. */
				xAddRequestHandler( pcName, xClickHTTPRequestHandler );
			}
			#endif

			xReturn = pdTRUE;
		}

		/* _CD_ Initialize Config for debugging */
		togglePins[0] = 0x01; //PA0
		togglePins[1] = 0x10; //PA4
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xExpand2Click_Deinit ( void )
{
BaseType_t xReturn = pdFALSE;

	if( xClickTaskHandle != NULL )
	{
		#if( includeHTTP_DEMO != 0 )
		{
			/* Use the task's name to remove the HTTP Request Handler. */
			xRemoveRequestHandler( pcTaskGetName( xClickTaskHandle ) );
		}
		#endif

		/* Delete the task. */
		vTaskDelete( xClickTaskHandle );
		/* Set the task handle to NULL, so the clickboard can be reactivated. */
		xClickTaskHandle = NULL;

		/* TODO: Reset I2C and GPIOs. */
		xReturn = pdTRUE;
	}
	return xReturn;
}
/*-----------------------------------------------------------*/

