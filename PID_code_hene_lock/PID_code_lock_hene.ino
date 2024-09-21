#include <Wire.h>
#include <PID_v1.h> #PID library
#include <Adafruit_ADS1X15.h>  #16 bit ADC library
Adafruit_ADS1115 ads;
 float Voltage = 0.0;
  float Voltage1 = 0.0;
const int numReadings =35;
//#define PIN_INPUT lightLevel
//#define PIN_INPUT 6

#define RELAY_PIN 6
const int led = 6; // LED output
double lightLevel;
int readings[numReadings];
int readings1[numReadings];// the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
float total1 = 0;                  // the running total
float average1 = 0;                // the average

float norm=0;
 float norm_pd1=0;
 float norm_pd2=0;
float err_sig=0;

double Setpoint, Input, Output;  //These are just variables for storing values
PID myPID(&Input, &Output, &Setpoint,3.6, .250551851154,0.044488081098, DIRECT); // This sets up our PID Loop
//Input is our PV
//Output is our u(t)
//Setpoint is our SP
const int sampleRate =1; // Variable that determines how fast our PID loop runs12
// Communication setup
const long serialPing =190;
unsigned long now = 0; //This variable is used to keep track of time
// placehodler for current timestamp
unsigned long lastMessage = -10; //
// seting the programmable gain amplifier (PGA)
void setup()
{
Serial.begin(9600);
//ads.setGain(GAIN_TWO);        +/- 2.048V  1 bit = 0.0625mV
//ads.setGain(GAIN_FOUR);      // +/- 1.024V  1 bit = 0.03125mV
//ads.setGain(GAIN_EIGHT);   //   +/- 0.512V  1 bit = 0.015625mV
ads.setGain(GAIN_SIXTEEN);    //+/- 0.256V  1 bit = 0.0078125mV  

ads.begin();

Setpoint =0;
myPID.SetMode(AUTOMATIC);  //Turn on the PID loop
  myPID.SetSampleTime(sampleRate); //Sets the sample rate
 lastMessage = millis();
}
 
void loop()
{
int16_t adc0, adc1, adc2, adc3;
  total = total - readings[readIndex];
  total1 = total1 - readings1[readIndex];
  // read from the sensor:
 readings[readIndex] = ads.readADC_SingleEnded(2);
  readings1[readIndex] = ads.readADC_SingleEnded(3);
  // add the reading to the total:
  total = total + readings[readIndex];
    total1 = total1 + readings1[readIndex];

  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  average1 = total1 / numReadings;
//adc0 = ads.readADC_SingleEnded(0);
Voltage =(average * 0.1875)/1000;
Voltage1 =(average1 * 0.1875)/1000;

norm = (Voltage+Voltage1);
  norm_pd1=(Voltage/norm)*8000;
  norm_pd2=(Voltage1/norm)*8000;
  err_sig=(norm_pd2-norm_pd1);

int Setpoint =0; //Read our setpoint
lightLevel = err_sig;
  
  //lightLevel = voltageA2; //Get the light level
  Input = lightLevel; //Map it to the right scale
  
  myPID.Compute();  //Run the PID loop
  analogWrite(6, Output); 
now = millis(); //Keep track of time
  if(now - lastMessage > serialPing) { 
//Serial.print("AIN0: ");

Serial.println( err_sig,5);

//Serial.print(" ");

//Serial.print(norm_pd2,5);

//Serial.print(" ");
//Serial.print(norm_pd1,5);

//Serial.println(" ");
delay(.05);
lastMessage = now; 
}}
