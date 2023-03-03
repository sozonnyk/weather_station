#include "secrets.h"

#define MY_DEBUG
#define MY_DEBUG_VERBOSE_TRANSPORT

#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_FREQUENCY RFM69_915MHZ
#define MY_RFM69_NETWORKID 101
#define MY_RFM69_ENABLE_ENCRYPTION
#define MY_ENCRYPTION_SIMPLE_PASSWD RFM69_PASSWD

#define MY_DEFAULT_LED_BLINK_PERIOD 20
#define MY_DEFAULT_TX_LED_PIN 9
#define MY_WITH_LEDS_BLINKING_INVERSE

#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <MCP79412RTC.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LTR390.h>

#define CHILD_ID_WIND 0
#define CHILD_ID_RAIN_DAY 1
#define CHILD_ID_RAIN_HOUR 2
#define CHILD_ID_LIGHT_LUX 3
#define CHILD_ID_UV_INDEX 4

#define SENSOR_RX 6
#define SENSOR_TX 7
#define RAIN_INPUT 3

#define SLEEP_TIME 60000 //ms
#define DWELL_TIME 100 //ms

#define RAIN_FACTOR 0.28 //mm per pulse

ModbusMaster node;
SoftwareSerial softSerial(SENSOR_RX, SENSOR_TX, false); // RX, TX // @suppress("Abstract class cannot be instantiated")
LTR390 ltr390(0x53);

int previousDay = 0;
int currentDay = 0;

int previousHour = 0;
int currentHour = 0;

volatile time_t time = 0;
volatile bool dayRainReceived = false;
volatile bool hourRainReceived = false;
volatile bool rtcSet = false;


volatile float rainDayValue = 0;
volatile float rainHourValue = 0;

#include <MySensors.h>

MyMessage windMsg(CHILD_ID_WIND, V_WIND);
MyMessage rainDayMsg(CHILD_ID_RAIN_DAY, V_RAIN);
MyMessage rainHourMsg(CHILD_ID_RAIN_HOUR, V_RAIN);
MyMessage luxMsg(CHILD_ID_LIGHT_LUX, V_LEVEL);
MyMessage uvMsg(CHILD_ID_UV_INDEX, V_UV);

extern char *__brkval;

int freeMemory() {
  char top;
  return &top - __brkval;
}


void onRainPulse() {
//	Serial.println("pulse");
	rainDayValue += RAIN_FACTOR;
	rainHourValue += RAIN_FACTOR;
}

float getUvi() {
	ltr390.setGain(LTR390_GAIN_18);   // Sensitivity reduction is nessesary
	ltr390.setResolution(LTR390_RESOLUTION_20BIT);
	ltr390.setMode(LTR390_MODE_UVS);

	do {
		wait(DWELL_TIME * 5);
	} while (!ltr390.newDataAvailable());

	float uvi = ltr390.readUVS(); //ltr390.getUVI();
//	Serial.print("UV: ");
//	Serial.println(uvi);
	return uvi;
}

float getLux() {
	ltr390.setGain(LTR390_GAIN_1); // Overflow in full sun if >1
	ltr390.setResolution(LTR390_RESOLUTION_20BIT);
	ltr390.setMode(LTR390_MODE_ALS);

	do {
		wait(DWELL_TIME * 5);
	} while (!ltr390.newDataAvailable());

	float lux = ltr390.getLux();
//	Serial.print("Lux: ");
//	Serial.println(lux);
	return lux;

}

void receiveTime(uint32_t ts) {
//	Serial.print("Time received: ");
//	Serial.println(ts);
	time = ts;
	RTC.set(time);
	rtcSet = true;
}

void receive(const MyMessage &message) {
	if (message.getType() == V_RAIN) {
//		Serial.print("sens: ");
//		Serial.println(message.getSensor());
		if (message.getSensor() == CHILD_ID_RAIN_DAY) {
			rainDayValue = message.getFloat();
			dayRainReceived = true;
//			Serial.print("Day received:");
//			Serial.println(rainDayValue);
		}
		if (message.getSensor() == CHILD_ID_RAIN_HOUR) {
			rainHourValue = message.getFloat();
			hourRainReceived = true;
//			Serial.print("Hour received:");
//			Serial.println(rainHourValue);
		}

	}
}

void presentation() {
	sendSketchInfo("Weather Station", "3.0");
	present(CHILD_ID_WIND, S_WIND, "Wind, km/h");
	present(CHILD_ID_RAIN_DAY, S_RAIN, "Rain, mm/day");
	present(CHILD_ID_RAIN_HOUR, S_RAIN, "Rain, mm/hour");
	present(CHILD_ID_LIGHT_LUX, S_LIGHT_LEVEL, "Sunlight, lux");
	present(CHILD_ID_UV_INDEX, S_UV, "UV Index");
}

float getRTU(uint16_t m_startAddress) {
	uint8_t m_length = 2;
	uint16_t result;
	result = node.readInputRegisters(m_startAddress, m_length);
	if (result == node.ku8MBSuccess) {
		return node.getResponseBuffer(0);
	} else {
		return 0;
	}
}

//Many defined strings in Serial.print eating memory, beware

void setup() {
//	Serial.begin(115200);
	ltr390.init();
	softSerial.begin(4800);
	node.begin(1, softSerial);

	pinMode(RAIN_INPUT, INPUT);
	attachInterrupt(digitalPinToInterrupt(RAIN_INPUT), onRainPulse, FALLING);

//	Serial.println("Waiting for time");

	RTC.begin();
	setSyncProvider(RTC.get);

	while (!rtcSet) {
		requestTime();
		wait(DWELL_TIME * 10);
	}
//
//	Serial.println("RTC set");
//	Serial.println("Waiting for count value");

	int tries = 0;

	request(CHILD_ID_RAIN_DAY, V_RAIN);
	wait(DWELL_TIME);
	request(CHILD_ID_RAIN_HOUR, V_RAIN);
	wait(DWELL_TIME);

	while (!hourRainReceived && !dayRainReceived && tries < 3) {
		tries++;
		wait(DWELL_TIME * 10);
	}

	currentDay = day();
	previousDay = currentDay;
	currentHour = hour();
	previousHour = currentHour;
}



void loop() {
//	Serial.print("Loop. Free: ");
//	Serial.println(freeMemory());
	setTime(RTC.get());

	int windRaw = getRTU(0x0000);
	float windSpeed = (windRaw / 10) * 3.6;
//	Serial.print("Speed: ");
//	Serial.print(windSpeed, 1);
//	Serial.println(" kmh ");
	send(windMsg.set(windSpeed, 1));


//	Serial.print("day: ");
//	Serial.println(currentDay);
//	Serial.println(previousDay);
//
//	Serial.print("hour: ");
//	Serial.println(currentHour);
//	Serial.println(previousHour);
//
//	Serial.print("val: ");
//	Serial.println(rainDayValue);
//	Serial.println(rainHourValue);

	currentDay = day();
	if (currentDay != previousDay) {
		rainDayValue = 0;
		previousDay = currentDay;
	}

	currentHour = hour();
	if (currentHour != previousHour) {
		rainHourValue = 0;
		previousHour = currentHour;
	}
//
	send(luxMsg.set(getLux(), 2));
	send(uvMsg.set(getUvi(), 0));
	send(rainDayMsg.set(rainDayValue, 2));
	send(rainHourMsg.set(rainHourValue, 2));

	wait(SLEEP_TIME);
}
