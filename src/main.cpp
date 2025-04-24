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
bool deviceConnected = false;

void testPrint();
void floatConversion();
void serialPlot();
uint8_t bytearr[12];
void printAdjustments();
void getAdjustments(uint8_t* arr);
void plotBuffer();

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		deviceConnected = true;
	};

	void onDisconnect(BLEServer* pServer) {
		deviceConnected = false;
		BLEDevice::startAdvertising();
	};
};


void setup() {
	Serial.begin(115200);
	
	BLEDevice::init("MyESP32");
  	BLEServer *pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());
  	
	BLEService *pService = pServer->createService(SERVICE_UUID); // BLE Service
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
	
	pService->start();
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x06); // 10ms
	pAdvertising->setMaxPreferred(0x12); // 20ms
	//pAdvertising->setMinInterval(0x06);   // 10ms
	//pAdvertising->setMaxInterval(0x12);   // 10ms
	BLEDevice::setMTU(247);
	BLEDevice::startAdvertising();

  	pCharacteristic->setValue("Blah");

	Serial.println("Characteristic defined!");



	// get divider parameters from client
	/*
	uint8_t* scales = pParamsCharacteristic->getData();
	int acc_scale = scales[0];
	int gyro_scale = scales[1];
	
	Serial.println("Received scales!");
	Serial.print("Acc_scale: "); Serial.println(acc_scale);
	Serial.print("Gyro_scale: "); Serial.println(gyro_scale);
	*/

	SPI.begin();
	delay(100);

	mpu.init(true, true);
	mpu.set_gyro_scale(0x18); // 2000DPS

	uint8_t wai = mpu.whoami(); // doesn't work? 
	if (wai == 0x71){
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}

	//mpu.calib_acc();
	//delay(100);

	// transmit the bias and scale parameters (have to be converted to floats later)
	uint8_t params[12];
	getAdjustments(params);
	pParamsCharacteristic->setValue(params, sizeof(params));
//	pParamsCharacteristic->notify(); 
	
	// printAdjustments();
	// For testing, expects FIFO setting
	// testPrint();
}

void loop() {
	mpu.read_fifo(); // updates fifo_data
			
	// for testing, run floatConversion, then plotBuffer.
	// read_all() with serialPlot also works
	//floatConversion();
	//serialPlot();
	//plotBuffer();
	
	if (deviceConnected) {
		pCharacteristic->setValue((uint8_t*)mpu.data_buffer, mpu.fifo_count);	
		pCharacteristic->notify();  // Send notification to connected device
	}

	delay(50);
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

// fifo data stored in big-endian format
	uint8_t* buffer = mpu.data_buffer;

	int16_t ax_buf[42], ay_buf[42], az_buf[42], gx_buf[42], gy_buf[42], gz_buf[42];

	for (int i =0 ; i<mpu.frameSize; i++) {
		ax_buf[i] = (((int16_t)buffer[i*12]) << 8) | buffer[i*12+1];  
		ay_buf[i] = (((int16_t)buffer[i*12+2]) << 8) | buffer[i*12+3];
		az_buf[i] = (((int16_t)buffer[i*12+4]) << 8) | buffer[i*12+5];

		gx_buf[i] = (((int16_t)buffer[i*12+6]) << 8) | buffer[i*12+7];
		gy_buf[i] = (((int16_t)buffer[i*12+8]) << 8) | buffer[i*12+9];
		gz_buf[i] = (((int16_t)buffer[i*12+10]) << 8) | buffer[i*12+11];
	}

	for(int n = 0; n < mpu.frameSize; n++) {
		mpu.ax_data[n] = ((float)ax_buf[n])/mpu.acc_divider - mpu.a_bias[0];
		mpu.ay_data[n] = ((float)ay_buf[n])/mpu.acc_divider - mpu.a_bias[1];
		mpu.az_data[n] = ((float)az_buf[n])/mpu.acc_divider - mpu.a_bias[2];
		mpu.gx_data[n] = ((float)gx_buf[n])/mpu.gyro_divider - mpu.g_bias[0];
		mpu.gy_data[n] = ((float)gy_buf[n])/mpu.gyro_divider - mpu.g_bias[1];
		mpu.gz_data[n] = ((float)gz_buf[n])/mpu.gyro_divider - mpu.g_bias[2];
	}
}


/* Writes to serial plotter after converting to float values */
void serialPlot() {		
	Serial.print(">accx:");
	Serial.println(mpu.accel_data[0]);
	Serial.print(">accy:");
	Serial.println(mpu.accel_data[1]);
	Serial.print(">accz:");
	Serial.println(mpu.accel_data[2]);

	Serial.print(">gyrox:");
	Serial.println(mpu.gyro_data[0]);
	Serial.print(">gyroy:");
	Serial.println(mpu.gyro_data[1]);
	Serial.print(">gyroz:");
	Serial.println(mpu.gyro_data[2]);
}

void plotBuffer() {
	for(int i = 0; i<mpu.frameSize; i++) {

		// Print every 10th sample
		if (i % 10 == 0) {
			Serial.print(">accx:");
			Serial.println(mpu.ax_data[i]);
			Serial.print(">accy:");
			Serial.println(mpu.ay_data[i]);
			Serial.print(">accz:");
			Serial.println(mpu.az_data[i]);
		
			Serial.print(">gyrox:");
			Serial.println(mpu.gx_data[i]);
			Serial.print(">gyroy:");
			Serial.println(mpu.gy_data[i]);
			Serial.print(">gyroz:");
			Serial.println(mpu.gz_data[i]);
		}
	}
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

	int16_t parameters[6] = {ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias};

	memcpy(empty_arr, parameters, sizeof(parameters)); // little-endian
}

void ble_connect() {	

}