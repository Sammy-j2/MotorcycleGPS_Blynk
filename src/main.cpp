#include <Arduino.h>

// credit to Great Scott for inspiration 
// Credit to Botletics for help on previous itieration via his board and arduino
// credit to ADXL library creator...
/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest
  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.
    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app
  Blynk library is licensed under MIT license
  This example code is in public domain.
 *************************************************************
  Attention! Please check out TinyGSM guide:
    https://tiny.cc/tinygsm-readme
  Change GPRS apm, user, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Fill-in your Template ID (only if using Blynk.Cloud) */
// #define BLYNK_TEMPLATE_ID ""
// #define BLYNK_DEVICE_NAME ""
// #define BLYNK_AUTH_TOKEN "";

#define BLYNK_TEMPLATE_ID "Enter your template ID here"
#define BLYNK_TEMPLATE_NAME "Enter your template name here"
#define BLYNK_AUTH_TOKEN "ENTER YOU AUTH TOKEN HERE"

// Select your modem:
#define TINY_GSM_MODEM_SIM7070

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
#define BLYNK_HEARTBEAT 60

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

#include <Arduino.h>
#include <Wire.h>
#include <ADXL345_WE.h>

char auth[] = "ENTER YOU AUTH TOKEN HERE";

#define ADXL345_I2CADDR 0x53     // ADXL345 adress byte
ADXL345_WE myAcc = ADXL345_WE(); // define name for accelerometer
const int alarmPin = 2;          // alarm pin
int alarmState = 0;

int percent;                     // percent variable for battery

BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[] = "SIM APN"; // GPRS APN
char user[] = "";
char pass[] = "";

#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PIN_RST 34
#define PWR_PIN 4
#define LED_PIN 12  
#define BAT_ADC 35
bool reply = false;

int GPSbutton;
int AlarmButton;
int batteryPercent;

float lat = 0.0, lon = 0.0;
double Lat = 0.0, Lon = 0.0;

double alarmSensitivity = 0.1;

TinyGsm modem(SerialAT);

WidgetMap myMap(V50);

BLYNK_WRITE(V3)
{
  int pinVal = param.asInt();

  if (pinVal == 1)
  {
    digitalWrite(LED_PIN, LOW);
  }

  else
  {
    digitalWrite(LED_PIN, HIGH);
  }
  Serial.println(pinVal);
}

BLYNK_WRITE(V4)
{

  alarmSensitivity = param.asDouble();

  myAcc.setActivityParameters(ADXL345_AC_MODE, ADXL345_XYZ, alarmSensitivity);

  Serial.print("alarmSensitivity: ");
  Serial.println(alarmSensitivity);
}

// Syncing the output state with the app at startup
BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);
  Blynk.syncVirtual(V12);
  Blynk.syncVirtual(V100);
}

////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////////////////////////////////

void alarmTrigger()
{

  myAcc.readAndClearInterrupts();
  delay(50);

  alarmState = digitalRead(alarmPin);

  if (alarmState == 1)
  {
    AlarmButton = 3;
    Serial.println("Motion Detected!!!");
    Blynk.virtualWrite(V101, "TRIGGERED!");
    Blynk.virtualWrite(V0, 1);
    Blynk.logEvent("alarm_triggered");
    GPSbutton = 1;

    
  }

}

void getGPS()
{
  Blynk.virtualWrite(V100, "Getting GPS...");
  if (modem.getGPS(&lat, &lon))
  {
    Serial.printf("lat:%f lon:%f\n", lat, lon);
  }
  if (lat != 0 && lon != 0)
  {
    Lat = (double)lat;
    Lon = (double)lon;
    Blynk.virtualWrite(V100, "GPS LOCKED");
    Blynk.virtualWrite(V10, Lon, Lat);
    // myMap.location(1, Lat, Lon, "value");
  }

  else
  {
    Serial.println("GETTING GPS...");
  }

  Blynk.virtualWrite(V11, Lat);
  Blynk.virtualWrite(V12, Lon);
}

BLYNK_WRITE(V0)
{

  GPSbutton = param.asInt();
}

BLYNK_WRITE(V1)
{
  AlarmButton = param.asInt();
}

void GPSswitch()
{

  switch (GPSbutton)
  {

  case 0:

    Blynk.virtualWrite(V100, "GPS OFF");

    Serial.printf("lat:%f lon:%f\n", 0.0, 0.0);
    Blynk.virtualWrite(V11, 0);
    Blynk.virtualWrite(V12, 0);

    GPSbutton = 99;

    break;

  case 1:

    getGPS();

    break;

  case 99:

    break;
  }
}

void AlarmSwitch()
{

  switch (AlarmButton)
  {

  case 0:

    Blynk.virtualWrite(V101, "OFF");

    AlarmButton = 99;

    break;

  case 1:

    alarmTrigger();
    Blynk.virtualWrite(V101, "ARMED");
    Serial.println("ARMED");

    AlarmButton = 98;

    break;

  case 3:
    Blynk.virtualWrite(V101, "Triggered!");
    Serial.println("TRIGGERED!!!");

    AlarmButton = 99;

    break;

  case 98:

    alarmTrigger();

    break;

  case 99:

    break;
  }
}

float readBattery(uint8_t pin)
{
  int vref = 1100;
  uint16_t volt = analogRead(pin);
  float battery_voltage = (volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

void batteryLevel()
{
  float mv = readBattery(BAT_ADC);
  Serial.print("mv :");
  Serial.println(mv);
  batteryPercent = ((mv / 4200) * 100);
  percent = map(batteryPercent,70,100,0,100);
  Blynk.virtualWrite(V2, percent);

  if (percent <= 80 && percent > 0 )
  {
    Blynk.logEvent("low_battery");
    Serial.println("battery runing low...");
  }
}

void connectionCheck()
{

  int tryCount = 0;
  while (modem.isGprsConnected() != 1)
  {
    Serial.println("GPRS disconected, trying to reconnect");
    tryCount++;
    
    Serial.println(F("restarting modem..."));
    delay(50);
    modem.restart();
    Serial.println(tryCount);
    vTaskDelay(2000);
    if (!modem.init())
    {
      Serial.println("Failed to initialize modem, attempting to continue with restarting");
      modem.restart();
    }
    vTaskDelay(500);

    if (tryCount == 3)
    {
      Serial.println("restarting ESP...");
      ESP.restart();
    }
  }
  if (!Blynk.connected())
  {
    vTaskDelay(3000);
    Blynk.connect();
  }
  
}

//////////////////////////////////////////// SET UP //////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200); // Set console baud rate
  delay(100);

  // Initialize ADXL345 Accelerometer

  Wire.begin();
  pinMode(alarmPin, INPUT);
  myAcc.init();
  if (!myAcc.init())
  {
    Serial.println("ADXL345 not connected!");
  }
  //  attachInterrupt(digitalPinToInterrupt(alarmPin), alarmTrigger, RISING);
  myAcc.setDataRate(ADXL345_DATA_RATE_3200);
  myAcc.setActivityParameters(ADXL345_AC_MODE, ADXL345_XYZ, alarmSensitivity);
  myAcc.setInterrupt(ADXL345_ACTIVITY, INT_PIN_2);

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);

  Serial.println("\nPlease Wait...");

  delay(1000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  while (!modem.init())
  {
    Serial.println("Failed to initialize modem, attempting to continue with restarting");
    modem.restart();
  }

  String name = modem.getModemName();
  delay(500);
  Serial.println("Modem Name: " + name);

  modem.sendAT("+SGPIO=0,4,1,1"); // Turns on active antenna for GPS
  while (modem.waitResponse(10000L) != 1)
  {
    Serial.println(" GPS NOT Powered ");
    delay(50);
    modem.sendAT("+SGPIO=0,4,1,1");
  }
  Serial.println("GPS Antenna Powered!");

  while (!modem.enableGPS()) // Turns on GPS
  {
    Serial.println("GPS not activacted, trying again...");
    delay(50);
  }

  Serial.println("GPS ACTIVATED!");

  Blynk.begin(auth, modem, apn, user, pass);

  connectionCheck();

  Blynk.virtualWrite(V1, 0);

  Blynk.logEvent("connected");

  batteryLevel();

  // Setup a functions to be called
  timer.setInterval(300000, batteryLevel);
  timer.setInterval(500L, AlarmSwitch);
  timer.setInterval(1000L, connectionCheck);
  timer.setInterval(3000L, GPSswitch);
}

//////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////////////////////////

void loop()
{

  Blynk.run();
  timer.run();
}