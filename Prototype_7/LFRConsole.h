#pragma once
#include <ArduinoJson.h>
// #include "config.h"
// #include "declarations.h"
#define JSON_MAX_LEN 196

void consoleAppTx(int sensorValues[], int pos, float error)
{
	JsonDocument consoleTX;
	char output[JSON_MAX_LEN];
	// printArr(sensorValuesDig, NUM_SENSORS + 1);
	// Serial.print("Digi");
	// String str = "";
	// for(int i = 0; i < NUM_SENSORS; i++) {
	//   Serial.print(sensorValuesDig[i]);
	//   if(sensorValuesDig[i] == '1') str = str + '1';
	//   else str += '0';
	// }
	// Serial.print(str);
	// consoleTX["di"] = String(sensorValuesDig);
	JsonArray data = consoleTX["an"].to<JsonArray>();
	for (int i = 0; i < sizeof(sensorValues) / sizeof(int); i++)
	{
		data.add(map(sensorValues[i], 0, 1000, 0, 99));
	}
	consoleTX["l1"] = "Error";
	consoleTX["v1"] = error;
	consoleTX["l2"] = "Position";
	consoleTX["v2"] = pos;

	serializeJson(consoleTX, output);
	Serial.print(output);
	// Serial.print(sensorValuesDig);
	// printArr(sensorValuesDig, NUM_SENSORS);
	Serial.println();
}
void consoleAppRx(char cmd, float pidValues[])
{
	JsonDocument consoleRX;
	Serial.flush();
	String consoleData = Serial.readStringUntil('\n');
	consoleData = cmd + consoleData;
	DeserializationError deSerError = deserializeJson(consoleRX, consoleData);
	deserializeJson(consoleRX, consoleData);
	if (deSerError)
	{
		Serial.print("Deserialize JSON FAILED: ");
		Serial.print(deSerError.f_str());
		Serial.println();
		consoleRX.clear();
		return;
	}

	pidValues[0] = consoleRX["P"];    // range 0-100
	pidValues[1] = consoleRX["I"];    // range 0-100
	pidValues[2] = consoleRX["D"];    // range 0-100
	pidValues[3] = consoleRX["ms"];   // range 0-255
	pidValues[4] = consoleRX["de"];   // range 0-50
}