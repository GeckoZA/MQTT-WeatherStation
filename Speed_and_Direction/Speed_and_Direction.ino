/*----- Include libraries -------*/
#include <math.h>
#include <Wire.h>

/*-----Comment for serial debugging-------- */
#define DEBUG true
//#define DEBUG false
/*------End of debigging----------- */


/* ------User Defined parameters----- */
#define SLAVE_ADDR 9
#define SpeedSensorPin (3)    // Digital pin for the wind Sensor
#define WSconversion 9.25     // Conversion factor for Rotations to Windspeed
#define WindVanePin (A0)      // The pin the wind vane sensor is connected to
/* ------End of User Defined Parameters------ */


volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
float Windspeedkm;
float WindDir;
char windspeedchar[10];
char winddirchar[10];

void setup() {
 Serial.begin(9600); // initialize Serial
 Wire.begin(SLAVE_ADDR);
 Wire.onRequest(requestEvent);

 pinMode(SpeedSensorPin, INPUT);
 attachInterrupt(digitalPinToInterrupt(SpeedSensorPin), isr_rotation, FALLING);
}

void loop(){
    
 Windspeedkm = returnWindSpeed();
 WindDir = returnWindDirection();

/*Converting float value to char
  The format (float, bytes, numbers of numbers after the decimal, char variable)*/
  dtostrf(WindDir, 5, 1, winddirchar); //Wind direction convertion
  dtostrf(Windspeedkm, 4, 2, windspeedchar); //Windspeed convertion

 
//If Debug is commented true 
  if (DEBUG) {
    Serial.println();
    Serial.print("Rotations");
    Serial.print(Rotations);
    Serial.println();
    Serial.print("Windspeed  ");
    Serial.print (Windspeedkm); 
    Serial.println(" Km/Hr");
    Serial.print("Wind Direction  ");
    Serial.print(WindDir);
    Serial.println(" deg Magnetic");;
    }
    Serial.println();
    Serial.println(windspeedchar);
    Serial.println(winddirchar);
    Serial.println();
    
}


//-----Working out the Windspeen ion Km/Hr------//
float returnWindSpeed(){
  
  float WindSpeed;    //Windspeed in Km/Hr
  
  Rotations = 0;      // Set Rotations count to 0 ready for calculations

  sei();              // Enables interrupts
  delay (3000);       // Wait 3 seconds to average
  cli();              // Disable interrupts
  
// convert to Km/hr using the formula WindSpeed (Km/hr) = Rotations * (conversion / Interupt Time Delay)
  //WindSpeed = Rotations * (WSconversion /3);
  return WindSpeed;
}


//------Workout the wind direction-------//
float returnWindDirection(){
  
  int VaneValue;// raw analog value from wind vane
  float Direction;// translated 0 - 360 direction
  float oldDirection = Direction;
  
    VaneValue = 85;
    //Serial.print(VaneValue);
// Map Analog values to cardinal headings
  if (VaneValue <= 797 && VaneValue >= 777){
    Direction = 0; 
  }
  else if (VaneValue <= 412 && VaneValue >= 400 && (oldDirection == 0 || oldDirection == 45)){
    Direction = 22.5;
  }
  else if (VaneValue <= 470 && VaneValue >= 455){
    Direction = 45;
  }
  else if (VaneValue <= 85 && VaneValue >= 70){
    Direction = 67.5;
  }
  else if (VaneValue <= 100 && VaneValue >= 70){
    Direction = 90;
  }
  else if (VaneValue <= 73 && VaneValue >= 60){
    Direction = 112.5;
  }
  else if (VaneValue <= 195 && VaneValue >= 175){
    Direction = 135;
  }
  else if (VaneValue <= 136 && VaneValue >= 116){
    Direction = 157.5;
  }
  else if (VaneValue <= 296 && VaneValue >= 276){
    Direction = 180;
  }
  else if (VaneValue <= 251 && VaneValue >= 231){
    Direction = 202.5;
  }
  else if (VaneValue <= 621 && VaneValue >= 601){
    Direction = 225;
  }
  else if (VaneValue <= 412 && VaneValue >= 400 && (oldDirection == 225 || oldDirection == 270)){
    Direction = 247.5;
  }
  else if (VaneValue <= 570 && VaneValue >= 550){
    Direction = 270;
  }
  else if (VaneValue <= 537 && VaneValue >= 517){
    Direction = 292.5;
  }
  else if (VaneValue <= 925 && VaneValue >= 905){
    Direction = 315;
  }
  else if (VaneValue <= 730 && VaneValue >= 710){
    Direction = 337.5;
  }
    return Direction;
}

//------ Data to send over I2C on request ------//
void requestEvent(){

  Wire.write(windspeedchar);                   // appx 5 bytes
  Wire.write(",");
  delay(10);
  Wire.write(winddirchar);                // appx 4 bytes
  Wire.write("\n");
 }

//------- This is the function that the interrupt calls to increment the rotation count -------//
void isr_rotation () {
 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

  if ((millis() - ContactBounceTime) > 15 ) // debounce the switch contact.
  { 
  Rotations++;
  ContactBounceTime = millis();
  }
}
