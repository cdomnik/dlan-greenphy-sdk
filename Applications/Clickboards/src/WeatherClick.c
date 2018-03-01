/*
 * WeatherClick.c
 *
 *  Created on: 18.12.2017
 *      Author: christoph.domnik
 */

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
#include "WeatherClick.h"
#include "save_config.h"

/* MQTT includes */
#include "mqtt.h"


/* Task handle used to identify the clickboard's task and check if the
clickboard is activated. */
static TaskHandle_t xClickTaskHandle = NULL;

bme280_calibration_param cal_param;
long adc_t, adc_p, adc_h, t_fine;

long Pressure;
unsigned long Temperature;
unsigned int Humidity;

float fTemp;
float fHum;
float fPress;

//--------------- Writes data to device - single location
static void I2C_WriteRegister(char wrAddr, char wrData) {
	I2C_XFER_T xfer;
	uint8_t tmp_data[2];

	tmp_data[0] = wrAddr;
	tmp_data[1] = wrData;

	xfer.slaveAddr = BME280_I2C_ADDRESS1;
	xfer.txBuff = tmp_data;
	xfer.txSz = 2;

	Chip_I2C_MasterSend( I2C1, xfer.slaveAddr, xfer.txBuff, xfer.txSz );
}

//--------------- Reads data from device - single location
static char I2C_ReadRegister(char rAddr) {
	I2C_XFER_T xfer;
	uint8_t tmp_data[2];

	tmp_data[0] = rAddr;

	xfer.slaveAddr = BME280_I2C_ADDRESS1;
	xfer.txBuff = tmp_data;
	xfer.txSz = 1;
	xfer.rxBuff = tmp_data;
	xfer.rxSz = 1;

	Chip_I2C_MasterSend( I2C1, xfer.slaveAddr, xfer.txBuff, xfer.txSz );
	Chip_I2C_MasterRead( I2C1, xfer.slaveAddr, xfer.rxBuff, xfer.rxSz );

	return tmp_data[0];
}


void BME280_ReadMeasurements() {
	I2C_XFER_T xfer;
	uint8_t tmp_data[BME280_DATA_FRAME_SIZE];

	tmp_data[0] = BME280_PRESSURE_MSB_REG;

	xfer.slaveAddr = BME280_I2C_ADDRESS1;
	xfer.txBuff = tmp_data;
	xfer.txSz = 1;
	xfer.rxBuff = tmp_data;
	xfer.rxSz = BME280_DATA_FRAME_SIZE;

	Chip_I2C_MasterSend( I2C1, xfer.slaveAddr, xfer.txBuff, xfer.txSz );
	Chip_I2C_MasterRead( I2C1, xfer.slaveAddr, xfer.rxBuff, xfer.rxSz );

	adc_h = tmp_data[BME280_DATA_FRAME_HUMIDITY_LSB_BYTE];
	adc_h |= (unsigned long)tmp_data[BME280_DATA_FRAME_HUMIDITY_MSB_BYTE] << 8;

	adc_t  = (unsigned long)tmp_data[BME280_DATA_FRAME_TEMPERATURE_XLSB_BYTE] >> 4;
	adc_t |= (unsigned long)tmp_data[BME280_DATA_FRAME_TEMPERATURE_LSB_BYTE] << 4;
	adc_t |= (unsigned long)tmp_data[BME280_DATA_FRAME_TEMPERATURE_MSB_BYTE] << 12;

	adc_p  = (unsigned long)tmp_data[BME280_DATA_FRAME_PRESSURE_XLSB_BYTE] >> 4;
	adc_p |= (unsigned long)tmp_data[BME280_DATA_FRAME_PRESSURE_LSB_BYTE] << 4;
	adc_p |= (unsigned long)tmp_data[BME280_DATA_FRAME_PRESSURE_MSB_BYTE] << 12;
}


char BME280_GetID() {
	return I2C_ReadRegister(BME280_CHIP_ID_REG);
}

void BME280_SoftReset() {
	I2C_WriteRegister(BME280_RST_REG, BME280_SOFT_RESET);
}

char BME280_GetStatus() {
	return I2C_ReadRegister(BME280_STAT_REG);
}

char BME280_GetCtrlMeasurement() {
	return I2C_ReadRegister(BME280_CTRL_MEAS_REG);
}

char BME280_GetCtrlHumidity() {
	return I2C_ReadRegister(BME280_CTRL_HUMIDITY_REG);
}

char BME280_GetConfig() {
	return I2C_ReadRegister(BME280_CONFIG_REG);
}


//Read factory calibration parameters
void BME280_ReadCalibrationParams() {
	char lsb, msb;

	msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG);
	cal_param.dig_T1 = (unsigned int) msb;
	lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG);
	cal_param.dig_T1 = (cal_param.dig_T1 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG);
	cal_param.dig_T2 = (int) msb;
	lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG);
	cal_param.dig_T2 = (cal_param.dig_T2 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG);
	cal_param.dig_T3 = (int) msb;
	lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG);
	cal_param.dig_T3 = (cal_param.dig_T3 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P1_MSB_REG);
	cal_param.dig_P1 = (unsigned int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P1_LSB_REG);
	cal_param.dig_P1 = (cal_param.dig_P1 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P2_MSB_REG);
	cal_param.dig_P2 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P2_LSB_REG);
	cal_param.dig_P2 = (cal_param.dig_P2 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P3_MSB_REG);
	cal_param.dig_P3 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P3_LSB_REG);
	cal_param.dig_P3 = (cal_param.dig_P3 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P4_MSB_REG);
	cal_param.dig_P4 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P4_LSB_REG);
	cal_param.dig_P4 = (cal_param.dig_P4 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P5_MSB_REG);
	cal_param.dig_P5 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P5_LSB_REG);
	cal_param.dig_P5 = (cal_param.dig_P5 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P6_MSB_REG);
	cal_param.dig_P6 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P6_LSB_REG);
	cal_param.dig_P6 = (cal_param.dig_P6 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P7_MSB_REG);
	cal_param.dig_P7 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P7_LSB_REG);
	cal_param.dig_P7 = (cal_param.dig_P7 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P8_MSB_REG);
	cal_param.dig_P8 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P8_LSB_REG);
	cal_param.dig_P8 = (cal_param.dig_P8 << 8) + lsb;

	msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P9_MSB_REG);
	cal_param.dig_P9 = (int) msb;
	lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P9_LSB_REG);
	cal_param.dig_P9 = (cal_param.dig_P9 << 8) + lsb;

	lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H1_REG);
	cal_param.dig_H1 = (char) lsb;

	msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG);
	cal_param.dig_H2 = (int) msb;
	lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG);
	cal_param.dig_H2 = (cal_param.dig_H2 << 8) + lsb;

	lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H3_REG);
	cal_param.dig_H3 = (char) lsb;

	msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG);
	cal_param.dig_H4 = (int) msb;
	lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG);
	cal_param.dig_H4 = (cal_param.dig_H4 << 4) | (lsb & 0xF);

	msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG);
	cal_param.dig_H5 = (int) msb;
	cal_param.dig_H5 = (cal_param.dig_H5 << 4) | (lsb >> 4);

	lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H6_REG);
	cal_param.dig_H6 = (short) lsb;
}

void BME280_SetOversamplingPressure(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK;
	ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS;

	I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingTemperature(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK;
	ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS;

	I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingHumidity(char Value) {

	I2C_WriteRegister(BME280_CTRL_HUMIDITY_REG, Value );
}

void BME280_SetOversamplingMode(char Value) {
	char ctrlm;
	ctrlm = BME280_GetCtrlMeasurement();
	ctrlm |= Value;

	I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetFilterCoefficient(char Value) {
	char cfgv;
	cfgv = BME280_GetConfig();
	cfgv &= ~BME280_CONFIG_REG_FILTER__MSK;
	cfgv |= Value << BME280_CONFIG_REG_FILTER__POS;
}

void BME280_SetStandbyTime(char Value) {
	char cfgv;
	cfgv = BME280_GetConfig();
	cfgv &= ~BME280_CONFIG_REG_TSB__MSK;
	cfgv |= Value << BME280_CONFIG_REG_TSB__POS;
}

char BME280_IsMeasuring() {
	char output;
	output = BME280_GetStatus();
	return (output & BME280_STAT_REG_MEASURING__MSK);
}

/****************************************************************************************************/
/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.  */
/***************************************************************************************************/

static long BME280_Compensate_T() {
	long temp1, temp2, T;

	temp1 = ((((adc_t>>3) -((long)cal_param.dig_T1<<1))) * ((long)cal_param.dig_T2)) >> 11;
	temp2 = (((((adc_t>>4) - ((long)cal_param.dig_T1)) * ((adc_t>>4) - ((long)cal_param.dig_T1))) >> 12) * ((long)cal_param.dig_T3)) >> 14;
	t_fine = temp1 + temp2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/************************************************************************************************************/
/* Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). */
/* Output value of “47445” represents 47445/1024 = 46.333 %RH */
/************************************************************************************************************/

static unsigned long BME280_Compensate_H() {
	long h1;
	h1 = (t_fine - ((long)76800));
	h1 = (((((adc_h << 14) - (((long)cal_param.dig_H4) << 20) - (((long)cal_param.dig_H5) * h1)) +
		((long)16384)) >> 15) * (((((((h1 * ((long)cal_param.dig_H6)) >> 10) * (((h1 *
		((long)cal_param.dig_H3)) >> 11) + ((long)32768))) >> 10) + ((long)2097152)) *
		((long)cal_param.dig_H2) + 8192) >> 14));
	h1 = (h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * ((long)cal_param.dig_H1)) >> 4));
	h1 = (h1 < 0 ? 0 : h1);
	h1 = (h1 > 419430400 ? 419430400 : h1);
	return (unsigned long)(h1>>12);
}

/***********************************************************************************************************/
/* Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa */
/***********************************************************************************************************/

static unsigned long BME280_Compensate_P() {
	long press1, press2;
	unsigned long P;

	press1 = (((long)t_fine)>>1) - (long)64000;
	press2 = (((press1>>2) * (press1>>2)) >> 11 ) * ((long)cal_param.dig_P6);
	press2 = press2 + ((press1*((long)cal_param.dig_P5))<<1);
	press2 = (press2>>2)+(((long)cal_param.dig_P4)<<16);
	press1 = (((cal_param.dig_P3 * (((press1>>2) * (press1>>2)) >> 13 )) >> 3) + ((((long)cal_param.dig_P2) * press1)>>1))>>18;
	press1 =((((32768+press1))*((long)cal_param.dig_P1))>>15);
	if (press1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	P = (((unsigned long)(((long)1048576)-adc_p)-(press2>>12)))*3125;
	if (P < 0x80000000) {
		P = (P << 1) / ((unsigned long)press1);
	} else {
		P = (P / (unsigned long)press1) * 2;
	}
	press1 = (((long)cal_param.dig_P9) * ((long)(((P>>3) * (P>>3))>>13)))>>12;
	press2 = (((long)(P>>2)) * ((long)cal_param.dig_P8))>>13;
	P = (unsigned long)((long)P + ((press1 + press2 + cal_param.dig_P7) >> 4));
	return P;
}


float BME280_GetTemperature() {
	return (float)BME280_Compensate_T() / 100;
}

float BME280_GetHumidity() {
	return (float)BME280_Compensate_H() / 1024;
}

float BME280_GetPressure() {
	return (float) BME280_Compensate_P() / 100;
}


static void vWeatherTask( void *pvParameters)
{
#if( netconfigUSEMQTT != 0 )
	/* Buffer for Publish Messages, need a buffer for each value,
	 * because buffer is given to mqtt task and may not be changed anymore
	 * until mqtt task sets it to NULL */
	char bufferTemp[7];
	char bufferHum[6];
	char bufferPress[8];

	/* Each Job and Publish Variable is needed for each Value because they will be send to the MqttQueue individually */
	QueueHandle_t xMqttQueue = xGetMQTTQueueHandle();
	MqttJob_t xTempJob;
	MqttJob_t xHumJob;
	MqttJob_t xPressJob;
	MqttPublishMsg_t xTempPub;
	MqttPublishMsg_t xHumPub;
	MqttPublishMsg_t xPressPub;

	/* Set all connection details for Temperature only on this place */
	xTempJob.eJobType = ePublish;
	xTempJob.data = (void *) &xTempPub;
	xTempPub.xMessage.qos = 0;
	xTempPub.xMessage.retained = 0;
	xTempPub.xMessage.payload = NULL;

	/* Set all connection details for Humidity only on this place */
	xHumJob.eJobType = ePublish;
	xHumJob.data = (void *) &xHumPub;
	xHumPub.xMessage.qos = 0;
	xHumPub.xMessage.retained = 0;
	xHumPub.xMessage.payload = NULL;

	/* Set all connection details for Pressure only on this place */
	xPressJob.eJobType = ePublish;
	xPressJob.data = (void *) &xPressPub;
	xPressPub.xMessage.qos = 0;
	xPressPub.xMessage.retained = 0;
	xPressPub.xMessage.payload = NULL;
#endif /* #if( netconfigUSEMQTT != 0 ) */
	for(;;)
	{
		BME280_SetOversamplingMode(BME280_FORCED_MODE);
		vTaskDelay( pdMS_TO_TICKS(500) );
		BME280_ReadMeasurements();

		fTemp = BME280_GetTemperature();
		fHum = BME280_GetHumidity();
		fPress = BME280_GetPressure();

		DEBUGOUT("WEATHER-CLICK: Temp:%.2f, Hum:%.2lf, Press:%.2f\n", fTemp, fHum, fPress);
	#if( netconfigUSEMQTT != 0 )
		xMqttQueue = xGetMQTTQueueHandle();
		if( xMqttQueue != NULL )
		{
			if(xTempPub.xMessage.payload == NULL)
			{
				/* _CD_ set payload each time, because mqtt task set payload to NULL, so calling task knows package is sent.*/
				xTempPub.xMessage.payload = bufferTemp;
				xTempPub.pucTopic = (char *)pvGetConfig( eConfigWeatherTempTopic, NULL );
				sprintf(bufferTemp, "%.2f", fTemp );
				xTempPub.xMessage.payloadlen = strlen(bufferTemp);
				xQueueSendToBack( xMqttQueue, &xTempJob, 0 );
			}

			if(xHumPub.xMessage.payload == NULL)
			{
				/* _CD_ set payload each time, because mqtt task set payload to NULL, so calling task knows package is sent.*/
				xHumPub.xMessage.payload = bufferHum;
				xHumPub.pucTopic = (char *)pvGetConfig( eConfigWeatherHumTopic, NULL );
				sprintf(bufferHum, "%.2f", fHum );
				xHumPub.xMessage.payloadlen = strlen(bufferHum);
				xQueueSendToBack( xMqttQueue, &xHumJob, 0 );
			}

			if(xPressPub.xMessage.payload == NULL)
			{
				/* _CD_ set payload each time, because mqtt task set payload to NULL, so calling task knows package is sent.*/
				xPressPub.xMessage.payload = bufferPress;
				xPressPub.pucTopic = (char *)pvGetConfig( eConfigWeatherPressTopic, NULL );
				sprintf(bufferPress, "%.2f", fPress );
				xPressPub.xMessage.payloadlen = strlen(bufferPress);
				xQueueSendToBack( xMqttQueue, &xPressJob, 0 );
			}
		}
	#endif /* #if( netconfigUSEMQTT != 0 ) */
		vTaskDelay( pdMS_TO_TICKS(500) );
	}
}

#if( includeHTTP_DEMO != 0 )
	static BaseType_t xClickHTTPRequestHandler(char *pcBuffer, size_t uxBufferLength, QueryParam_t *pxParams, BaseType_t xParamCount)
	{
		BaseType_t xCount = 0;
		QueryParam_t *pxParam;

		pxParam = pxFindKeyInQueryParams( "weatherTemp", pxParams, xParamCount );
		if( pxParam != NULL )
			pvSetConfig( eConfigWeatherTempTopic, strlen(pxParam->pcValue) + 1, pxParam->pcValue );

		pxParam = pxFindKeyInQueryParams( "weatherHum", pxParams, xParamCount );
		if( pxParam != NULL )
			pvSetConfig( eConfigWeatherHumTopic, strlen(pxParam->pcValue) + 1, pxParam->pcValue );

		pxParam = pxFindKeyInQueryParams( "weatherPress", pxParams, xParamCount );
		if( pxParam != NULL )
			pvSetConfig( eConfigWeatherPressTopic, strlen(pxParam->pcValue) + 1, pxParam->pcValue );

		xCount += sprintf( pcBuffer, "{\"wtemp\":%.2f,\"whum\":%.2f,\"wpress\":%.2f", fTemp, fHum, fPress );

	#if( netconfigUSEMQTT != 0 )
		char buffer[50];
		char *pcTempTopic = (char *)pvGetConfig( eConfigWeatherTempTopic, NULL );
		char *pcHumTopic =  (char *)pvGetConfig( eConfigWeatherHumTopic, NULL );
		char *pcPressTopic =  (char *)pvGetConfig( eConfigWeatherPressTopic, NULL );
		if( pcTempTopic != NULL )
		{
			strcpy( buffer, pcTempTopic );
			vCleanTopic( buffer );
			xCount += sprintf( pcBuffer + xCount , ",\"weatherTemp\":\"%s\"", buffer );
		}

		if( pcHumTopic != NULL )
		{
			strcpy( buffer, pcHumTopic );
			vCleanTopic( buffer );
			xCount += sprintf( pcBuffer + xCount , ",\"weatherHum\":\"%s\"", buffer );
		}

		if( pcPressTopic != NULL )
		{
			strcpy( buffer, pcPressTopic );
			vCleanTopic( buffer );
			xCount += sprintf( pcBuffer + xCount , ",\"weatherPress\":\"%s\"", buffer );
		}
	#endif /* #if( netconfigUSEMQTT != 0 ) */

		xCount += sprintf( pcBuffer + xCount , "}" );
		return xCount;
	}
#endif /* #if( includeHTTP_DEMO != 0 ) */

BaseType_t xWeatherClick_Init ( const char *pcName, BaseType_t xPort )
{
BaseType_t xReturn = pdFALSE;
char tmp;

	/* Use the task handle to guard against multiple initialization. */
	if( xClickTaskHandle == NULL )
	{
		DEBUGOUT( "Initialize WeatherClick on port %d.\r\n", xPort );

		/* Initialize I2C. Both microbus ports are connected to the same I2C bus. */
		Board_I2C_Init( I2C1 );
		if( xSemaphoreTake( xI2C1_Mutex, portMAX_DELAY ) == pdTRUE )
		{
			/* Initialize WeatherClick chip. */
			tmp = BME280_GetID();
			if (tmp != BME280_CHIP_ID) {
				DEBUGOUT("Weather-Click Error 0x0010: Chip Error!");
				return pdFALSE;
			}
			//Read calibration parameters
			BME280_ReadCalibrationParams();
			// Set Oversampling to 1 (recomended for environmental measurements )
			BME280_SetOversamplingPressure(BME280_OVERSAMP_1X);
			BME280_SetOversamplingTemperature(BME280_OVERSAMP_1X);
			BME280_SetOversamplingHumidity(BME280_OVERSAMP_1X);

			/* Give Mutex back, so other Tasks can use I2C */
			xSemaphoreGive( xI2C1_Mutex );

			/* Create task. */
			xTaskCreate( vWeatherTask, pcName, 320, NULL, ( tskIDLE_PRIORITY + 1 ), &xClickTaskHandle );
		}

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
	}

	return xReturn;
}



BaseType_t xWeatherClick_Deinit ( void )
{
BaseType_t xReturn = pdFALSE;

	if( xClickTaskHandle != NULL )
	{
		DEBUGOUT( "Deinitialize WeatherClick.\r\n" );

		/*#if( includeHTTP_DEMO != 0 )
		{
			/* Use the task's name to remove the HTTP Request Handler. */
			/*xRemoveRequestHandler( pcTaskGetName( xClickTaskHandle ) );
		}
		#endif*/

		/* Delete the task. */
		vTaskDelete( xClickTaskHandle );
		/* Set the task handle to NULL, so the clickboard can be reactivated. */
		xClickTaskHandle = NULL;

		xReturn = pdTRUE;
	}
	return xReturn;
}
