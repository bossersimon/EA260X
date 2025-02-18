/**
 * Sample program for the MPU9250 using SPI (MODIFIED)
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT license. See LICENSE.txt.
 */

#include <SPI.h> // From the Arduino core
#include "MPU9250.h"




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

void setup() {
	Serial.begin(115200);

//	pinMode(INT_PIN, INPUT); // Interrupt 
//	pinMode(LED, OUTPUT);
//	digitalWrite(LED, HIGH);

	SPI.begin();

	Serial.println("Press any key to continue");
	WAITFORINPUT();

	mpu.init(true);

	uint8_t wai = mpu.whoami();
	if (wai == 0x71){
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}

	mpu.calib_acc();

	Serial.println("Send any char to begin main loop.");
	WAITFORINPUT();
}

void loop() {

	mpu.read_acc();
	mpu.read_gyro();
	//mpu.read_all();

	// send to serial plotter
	Serial.print(">gyrox:");
	Serial.println(mpu.gyro_data[0]);
	Serial.print(">gyroy:");
	Serial.println(mpu.gyro_data[1]);
	Serial.print(">gyroz:");
	Serial.println(mpu.gyro_data[2]);

	Serial.print(">accx:");
	Serial.println(mpu.accel_data[0]);
	Serial.print(">accy:");
	Serial.println(mpu.accel_data[1]);
	Serial.print(">accz:");
	Serial.println(mpu.accel_data[2]);



	delay(10);
}
