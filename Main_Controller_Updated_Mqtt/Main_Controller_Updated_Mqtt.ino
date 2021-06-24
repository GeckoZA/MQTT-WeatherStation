 /*----- Include libraries -------*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_VEML6070.h"
#include <BH1750.h>

//---------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------Config Section-----------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------//

/*-----------------------------------------Comment for serial debugging----------------------------------------------------- */
#define DEBUG true
//#define DEBUG false

/* -----------------------------------------------User Defined Pins and Times------------------------------------------------*/
#define RAINPIN 13                  // Rain sensor pin
#define interval  20                //Rain Rate reset time in min
#define cleartime  24               //Total Rain fall clear time
/*----------------------------------------- WiFi and MQTT server parameters -------------------------------------------------*/
#define wifi_ssid "SSID"
#define wifi_password "WIFI PASSWORD"
#define mqtt_server "SERVER IP ADDRESS"
#define mqtt_user "username"              // MQTT username
#define mqtt_password "password"          //MQTT Password
#define PUBLISH_TIME 30                   // Time in seconds to publish mqtt messages
#define WIND_TIME 10                      // Time in seconds to publish wind information mqtt messages

/*---------------------------------------------- MQTT topic config --------------------------------------------------------- */
#define windspeed_topic "WeatherStation/windspeed"              //Topic windspeed Km/hr
#define winddirection_topic "WeatherStation/winddirection"      //Topic Winddirection deg
#define temperature_topic "WeatherStation/temperature"          //Topic Temperature deg C
#define pressure_topic "WeatherStation/pressure"                //Topic Pressure hPa
#define humidity_topic "WeatherStation/humidity"                //Topic Humidity %
#define heatindex_topic "WeatherStation/heatindex"              //Topic Heat Index deg C
#define dewpoint_topic "WeatherStation/dewpoint"                //Topic Dew Point deg C
#define lux_topic "WeatherStation/lux"                          //Topic Lux
#define windchill_topic "WeatherStation/windchill"              //Topic Wind Chill deg C
#define rainfall_topic "WeatherStation/rainfall"                //Topic Rain Fall in mm pertime period
#define uvi_topic "WeatherStation/uv"                           //Topic Rain Fall in Raw UVA
#define vox_topic "WeatherStation/vox"                          //Topic Rain Fall in resistance
#define altitude_topic "WeatherStation/altitude"                //Topic Rain Fall in pressure altitude in meters
#define rainrate_topic "WeatherStation/rainrate"                //Topic Rain Fall in mm/hr

//---------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------End Config Section-------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------//

#define SEALEVELPRESSURE_HPA (1013.25)

volatile unsigned long Tippings; // cup tipping counter used in interrupt routine
long lastMsg = 0;
unsigned long previousMillis = 0;
long lastWind = 0;
unsigned long previousWind = 0;
volatile unsigned long tiptime = millis();
volatile unsigned long tipcount = 0;
unsigned long previousClear = 0;
float temperature;         // in deg C
float humidity;            // in %
int pressure;            // in hPa
int windspeed;             // in Km/hr
int rotations;             // Rotations per 5 sec
float winddirection;       // in deg
float dewpoint;            // in degC
float heatindex;           // in degC
float lux;                 // in Lux
float windchill;           // in degC
float windGust;            // in km/hr 
float rainfall;            // in mm in 24hr
float rainrate;            // in mm/hr
float vox;                 // in resistance
uint16_t uvi;              // in UV A units
int altitude;              // in meters
int uvIndex;               // UV Index
float intdirection;


  
//Create instances for the sensors
WiFiClient espClient;
PubSubClient client(espClient);
BH1750 lightMeter;
Adafruit_BME680 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();

//------- Setup Function -------//
void setup() {
  Wire.begin();         // join i2c bus 
  Serial.begin(9600);   //Start Serial
  setup_wifi();           //Connect to Wifi network
  client.setServer(mqtt_server, 1883);    // Configure MQTT connexion
    
 
  lightMeter.begin();
  uv.begin(VEML6070_4_T);  // pass in the integration time constant

if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
// Set up oversampling and filter initialization on BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

 pinMode(RAINPIN, INPUT);
 attachInterrupt(digitalPinToInterrupt(RAINPIN), isr_rain, FALLING);
}


//-------- Setup and connect to WiFi ---------//
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi Connected ");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
}

//-------- Reconnection --------//
void reconnect() {

  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker ...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("OK");
    } else {
      Serial.print("Error : ");
      Serial.print(client.state());
      Serial.println(" Waiting 5 secondes before retrying");
      delay(5000);
    }
  }
}

//------- Loop Function -------//
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  getWindInfo();
  windGust    = calcwindGust();
  calcWindspeed();
  
 // Sensor and Formula Data
  temperature = (bme.temperature);
  humidity    = (bme.humidity);
  pressure    = (bme.pressure / 100.0);
  vox         = (bme.gas_resistance / 1000.0);
  altitude    = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  dewpoint    = temperature - ((100 - humidity) / 5);
  heatindex   = getHeatIndex();
  lux         = lightMeter.readLightLevel();
  windchill   = returnWindChill();
  uvi         = uv.readUV();
  rainfall    = returnRainFall(); 
  uvIndex     = getUVIndex();

 // Publish All Data over MQTT
  mqttPublish();          
  windPublish();
  resetRainRate();
  
  //If Debug is commented true in Config
  if (DEBUG) {
    Serial.println();
    Serial.println("Temp\tPressure\tHumidity\tHeat Index\tDew Point\tLux   \t\tSpeed(Km/hr)\tDirection\tWind Chill\tRainfall\tUV");
    Serial.print (temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t\t");
    Serial.print(humidity); Serial.print("\t\t");
    Serial.print(heatindex); Serial.print("\t\t");
    Serial.print(dewpoint); Serial.print("\t\t");
    Serial.print(lux); Serial.print("   \t");
    Serial.print(windspeed);Serial.print("\t\t");
    Serial.print(winddirection);Serial.print("\t\t");
    Serial.print(windchill);Serial.print("\t\t");
    Serial.print(rainrate);Serial.print("\t");
    Serial.print(rainfall);Serial.print("\t");
    Serial.print(uvi);Serial.print("\t");    
    }
}


//------- MQTT publish to Weather Station function ----------//
void mqttPublish(){
    
  long now = millis();
  // Send a message every minute
  if (now - lastMsg > 1000 * PUBLISH_TIME) {
    lastMsg = now;
    
      client.publish(temperature_topic, String(temperature).c_str(), true);          // Publish Temperature deg/C
      client.publish(pressure_topic, String(pressure).c_str(), true);                // Publish Pressure hPa
      client.publish(humidity_topic, String(humidity).c_str(), true);                // Publish Humidity %
      client.publish(heatindex_topic, String(heatindex).c_str(), true);              // Publish Heat index deg/C
      client.publish(dewpoint_topic, String(dewpoint).c_str(), true);                // Publish Dew Point deg/C
      client.publish(lux_topic, String(lux).c_str(), true);                          // Publish lux
      client.publish(windchill_topic, String(windchill).c_str(), true);              // Publish Wind Chill deg/C
      client.publish(vox_topic, String(vox).c_str(), true);                          // Publish Volitile organic Compounds
      client.publish(altitude_topic, String(altitude).c_str(), true);                // Publish Pressure Altitude
      client.publish(uvi_topic, String(uvi).c_str(), true);                          // Publish UV Level
    } 
}

//------ Get Wind speed and direction over I2c -------//
void getWindInfo(){
  Wire.requestFrom(9, 10);                // request 10 bytes from slave device #9

  String string, speed, direction;        //Create string for the incomming I2C char array
  do
  {
    char c = Wire.read();                 // receive a byte as character

    string = string + c;                  //Keep saving whatever is comming
    
    //********Checking of full string msg
    speed = string.substring(0, 2);       //slpit String from 0 to 4
    direction = string.substring(3, 8);   //slpit String from 5 to 9
  }
    while (Wire.available());             // slave may send less than requested

 rotations = speed.toInt();               //Convert speed string to float
 winddirection = direction.toFloat();     //Convert direvtion string to int
}


//------------Function to calculate the wind speed based on rotations--------------//
int calcWindspeed(){

 if (rotations = 0 || rotations <=8){
 windspeed = rotations * 1.98;
 }
 else if (rotations >= 9 || rotations <=17){
  windspeed = rotations * 1.92;
 }
 else if (rotations >= 18 || rotations <=30){
  windspeed = rotations * 1.3;
 }
 else if (rotations >= 31 || rotations <=50){
  windspeed = rotations * 1.28;
 }
else if (rotations >= 51 || rotations <=60){
  windspeed = rotations * 1.25;
 }
 else if (rotations >= 61){
  windspeed = rotations * 1.21;
 }
return windspeed;
}


void windPublish(){

long now = millis();
  // Send a message every minute
  if (now - lastWind > 1000 * WIND_TIME) {
    lastWind = now;
  }
    client.publish(windspeed_topic, String(windspeed).c_str(), true);              // Publish windspeed km/hr
    client.publish(winddirection_topic, String(winddirection).c_str(), true);      // Publish wind direction deg
}

//-------Function for Windchill in degC -------//
float returnWindChill(){
  float chill;
  float V = windspeed;
  float T = temperature;
  
  if (temperature < 15 && windspeed > 5)
    chill = 13.12 + 0.6215*T - 11.37 * (pow(V,0.16 )) + 0.3965*T * (pow(V,0.16 )) ;   //Metric formula wor wind chill
  else
    chill = temperature;
    
  return chill;
}

//-------Fuction to calculate Wind Gust value---------//
float calcwindGust(){
  float windGustCalc = 0;
  float currentWS = windspeed;
  float oldWS = 0;
  int differance = 10;
  int timeFrame = 10;

//**********Add a time factor and average*********//////

  if (currentWS > oldWS){
    oldWS = currentWS;
  }
  return windGustCalc;
}

//-------Function for Heat index Value in degC---------//
float getHeatIndex(){
  float heatindexC;
  float hi;
  float tempF;
  float tempC = temperature;
  
tempF = (tempC * 1.8) + 32;                                             // Confert Celcius to Farinheit
  
hi = 0.5 * (tempF + 61.0 + ((tempF - 68.0) * 1.2) + (humidity * 0.094));  //Basic heat index formula for temp below 79F or 26C
  
  if (hi > 79) {                                                          //Advance Formula for temp above 79F or 26C
    hi = -42.379 + 2.04901523 * tempF + 10.14333127 * humidity +          //HI= c1+c2T+c3R+c4TR+c5T2+c6R2+c7T2R+c8TR2+c9T2R2
         -0.22475541 * tempF * humidity +
         -0.00683783 * pow(tempF, 2) +
         -0.05481717 * pow(humidity, 2) +
         0.00122874 * pow(tempF, 2) * humidity +
         0.00085282 * tempF * pow(humidity, 2) +
         -0.00000199 * pow(tempF, 2) * pow(humidity, 2);
          
    if ((humidity < 13) && (tempF >= 80.0) && (tempF <= 112.0))           //Reduction in Heatindex based on Humidity and Temperature
      hi -= ((13.0 - humidity) * 0.25) * sqrt((17.0 - abs(tempF - 95.0)) * 0.05882);

    else if ((humidity > 85.0) && (tempF >= 80.0) && (tempF <= 87.0))     //Addition in Heat Index based on Humidity and Temperature
      hi += ((humidity - 85.0) * 0.1) * ((87.0 - tempF) * 0.2);
  }
heatindexC = (hi - 32) /1.8;                                              //Confert Heat Index from Farenheit to Celcius
  
return heatindexC;      
}

//-------- Function to convert Raw UV to a UV index-------//
// not complete
int getUVIndex(){
  /*int uvindex;

  if ( uvi >= 0 && uvi < 689){
    uvindex = 
  }*/
}


//------ Fuction to reset Rainrate after a giver interval------//
void resetRainRate(){

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= (interval * 1000 * 60)){        // interval set in config section
    previousMillis = currentMillis;
    
    rainrate = 0;
  }
}


//------ Function to work out Rainfall in a set time period -------//
float returnRainFall(){
  
float rainFall;
unsigned long now = millis();

if (now - previousClear >= (cleartime * 3600000)){                  // Cleartime = the time in hours set in config section
  previousClear = now;
  client.publish(rainfall_topic, String(rainfall).c_str(), true);      // Publish total Rain Fall in mm from set clear time               
  Tippings = 0;
    }
 else{
  rainFall = Tippings * 0.768;
 }
 return rainFall;
}

//------ Rain Sensor tipping interupt ------//
ICACHE_RAM_ATTR void isr_rain () {
 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

  if ((millis() - ContactBounceTime) > 15 ) // debounce the switch contact.
  { 
  Tippings++;
  ContactBounceTime = millis();
  }
  
  tipcount = millis() - tiptime;
  tiptime = millis();
  
  rainrate = 0.768*3600000 / tipcount;
  client.publish(rainrate_topic, String(rainrate).c_str(), true);                // Publish Rain Fall mm/hr
}
