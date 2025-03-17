/**
 * Sample program for the MPU9250 using SPI (MODIFIED)
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT license. See LICENSE.txt.
 */

#include <SPI.h> // From the Arduino core
#include "MPU9250.h"

#include <typeinfo>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID        "9fc7cd06-b6aa-492d-9991-4d5a433023e5"
#define CHARACTERISTIC_UUID "c1756f0e-07c7-49aa-bd64-8494be4f1a1c"
#define PARAMS_CHARACTERISTIC_UUID "97b28d55-f227-4568-885a-4db649a8e9fd"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   5 // Slave select
//#define INT_PIN  3  // Interrupt
//#define LED      13 // not needed? 

// Replace with inline?
#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  \

MPU9250 mpu(SPI_CLOCK, SS_PIN);
BLECharacteristic *pCharacteristic;
BLECharacteristic *pParamsCharacteristic;


void testPrint();
void floatConversion();
void serialPlot();
uint8_t bytearr[12];
void printAdjustments();
void getAdjustments(uint8_t* arr);

void setup() {
	Serial.begin(115200);

	SPI.begin();
	delay(100);

	mpu.init(true);

	uint8_t wai = mpu.whoami(); // doesn't work? 
	if (wai == 0x71){
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}

	mpu.calib_acc();
	mpu.init_fifo(); // Enables buffering to FIFO

	
	BLEDevice::init("MyESP32");
  	BLEServer *pServer = BLEDevice::createServer();
  	
	BLEService *pService = pServer->createService(SERVICE_UUID);
	pCharacteristic = pService->createCharacteristic(
    	CHARACTERISTIC_UUID,
    	BLECharacteristic::PROPERTY_NOTIFY
  	);

	pParamsCharacteristic = pService->createCharacteristic(
    	PARAMS_CHARACTERISTIC_UUID,
    	BLECharacteristic::PROPERTY_NOTIFY |
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_WRITE
  	);

	pCharacteristic->addDescriptor(new BLE2902());
	pParamsCharacteristic->addDescriptor(new BLE2902());
	
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x06); // 10ms
	pAdvertising->setMinPreferred(0x12); // 20ms
	//pAdvertising->setMinInterval(0x06);   // 10ms
	//pAdvertising->setMaxInterval(0x12);   // 10ms
	//BLEDevice::setMTU(517);
	BLEDevice::startAdvertising();

  	pCharacteristic->setValue("Blah");
  	pService->start();

	Serial.println("Characteristic defined!");

	// get divider parameters from client
	uint8_t* scales = pParamsCharacteristic->getData();
	int acc_scale = scales[0];
	int gyro_scale = scales[1];

	mpu.set_acc_scale(acc_scale);
	mpu.set_gyro_scale(gyro_scale);

	// transmit the bias and scale parameters (have to be converted to floats later)
	uint8_t params[12];
	getAdjustments(params);
	pParamsCharacteristic->setValue(params, sizeof(params));
	
	printAdjustments();
	// For testing, expects FIFO setting
	// testPrint();

	delay(1000);
}

void loop() {

	mpu.read_fifo(); // updates fifo_data

	pCharacteristic->setValue((uint8_t*)mpu.fifo_data, sizeof(mpu.fifo_data));	
  	pCharacteristic->notify();  // Send notification to connected device
	delay(30);

	// for testing
	//floatConversion();
	//serialPlot();
}

/* Prints one reading to be transmitted (for testing) */
void testPrint() {
	delay(1000);
	mpu.ReadRegs(MPUREG_FIFO_R_W, bytearr, 12); // read FIFO once
	for (int i = 0; i<12; i+=2) {
		int16_t val = ((int16_t)bytearr[i] << 8 | bytearr[i+1]); // big endian
		Serial.print(val);
		Serial.print(", ");
	}

	while (true) {
		pCharacteristic->setValue((uint8_t*)bytearr, sizeof(bytearr));	
		pCharacteristic->notify();  // Send notification to connected device
		delay(1000);
	}
}

/*	 This is for converting coordinates to float values, for testing purposes */
void floatConversion() {

	// fifo data stored as [ax,ay,az,gx,gy,gz] big-endian format
	if ( mpu.fifo_data ) {
		int16_t bit_data;
		float data;
		for(int i = 0; i < 3; i++) {
			bit_data = ((int16_t)mpu.fifo_data[i * 2] << 8) | mpu.fifo_data[i * 2 + 1];
				data = (float)bit_data;
				mpu.accel_data[i] = data / mpu.acc_divider - mpu.a_bias[i];

				bit_data = ((int16_t)mpu.fifo_data[6 + i * 2] << 8) | mpu.fifo_data[6 + i * 2 + 1];
				data = (float)bit_data;
				mpu.gyro_data[i] = data / mpu.gyro_divider - mpu.g_bias[i];
		}
	}
}


/* Writes to serial plotter after converting to float values */
void serialPlot() {
	float ax = mpu.accel_data[0];
	float ay = mpu.accel_data[1];
	float az = mpu.accel_data[2];
	float gx = mpu.gyro_data[0];
	float gy = mpu.gyro_data[1];
	float gz = mpu.gyro_data[2];
		
	Serial.print(">gyrox:");
	Serial.println(gx);
	Serial.print(">gyroy:");
	Serial.println(gy);
	Serial.print(">gyroz:");
	Serial.println(gz);

	Serial.print(">accx:");
	Serial.println(ax);
	Serial.print(">accy:");
	Serial.println(ay);
	Serial.print(">accz:");
	Serial.println(az);
}

void printAdjustments() {
	Serial.print("a_bias: "); Serial.print(mpu.a_bias[0]); Serial.print(" "); 
	Serial.print(mpu.a_bias[1]); Serial.print(" "); Serial.println(mpu.a_bias[2]);

	Serial.print("g_bias: "); Serial.print(mpu.g_bias[0]); Serial.print(" "); 
	Serial.print(mpu.g_bias[1]); Serial.print(" "); Serial.println(mpu.g_bias[2]);

	//Serial.print("acc_divider: "); Serial.println(mpu.acc_divider);
	//Serial.print("gyro_divider: "); Serial.println(mpu.gyro_divider);
}

void getAdjustments(uint8_t* empty_arr) {
	int16_t ax_bias = (int16_t)(mpu.a_bias[0]*100);
	int16_t ay_bias = (int16_t)(mpu.a_bias[1]*100);
	int16_t az_bias = (int16_t)(mpu.a_bias[2]*100);
	int16_t gx_bias = (int16_t)(mpu.g_bias[0]*100);
	int16_t gy_bias = (int16_t)(mpu.g_bias[1]*100);
	int16_t gz_bias = (int16_t)(mpu.g_bias[2]*100);

	//int16_t gyro_div = (int16_t)(mpu.gyro_divider*100);
	//int16_t acc_div = mpu.acc_divider*100;

	int16_t parameters[6] = {ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias};
	memcpy(empty_arr, parameters, sizeof(parameters)); // little-endian
}