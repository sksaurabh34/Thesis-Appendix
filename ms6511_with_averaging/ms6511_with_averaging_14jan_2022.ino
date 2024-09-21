#include <MS5611.h>

#include "MS5611_newlib.h"

MS5611_newlib ms(0x77); // or 0x76
const int numReadings = 40;

float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
float temp =0;
void setup() {
  Serial.begin(9600);
   // initialize serial communication with computer:
  // initialize all the readings to 0:
  delay(1500);
  int ret = ms.begin();

  if(ret!=0) Serial.println("Device not found");
for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}
void loop() {
   float temp = ms.measure_temp(OSR_4096);
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] =temp;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  Serial.println(average,4);
  delay(500);        // delay in between reads for stability
}
