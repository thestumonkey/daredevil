
// Include system headers
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#include <SPI.h>
#include <i2c.h>


// Debug
#include "debug.h"

// Include our headers
#include "Metro.h"
#include "motor.h"

// Init timer for 2s, and auto-reset when we get a positive check
Metro timer( 2000, 1 );
Motor motor = Motor();

int maximumRange = 200;             // Maximum range needed
int minimumRange = 0;               // Minimum range needed
int distancegroup, measurecount;

void setup()
{
	// Setup serial
	Serial.begin( 9600 );
	Serial.print( F("FreeMem=") );
	Serial.println( freeRAM() );
	
	setupPins();
	i2cInit( 200 );
	
	// Set up the motor
	motor.selectMotor( 2 );
	motor.autoCalibrate();

	// Ensure any time for calibration is ignored.
	timer.reset();
}

void loop()
{
//  Serial.println("-Measuring distance");
  long distance = measuredistance();
  printdistance(distance);
//  Serial.println("-Getting distance group");
//  getdistancegroup(distance);
//  Serial.println("-Playing effect");
//  playeffect();
//  long long_distance = getLongDistance();
 
  delay(500);
  
}

void setupPins()
{
	pinMode( SW_LRA_M,		OUTPUT );	// Output for switching LRA+/- v.s. M+/-
	pinMode( SW_MOS_DRV,	        OUTPUT );	// Output for switching +3.3V/MOS- v.s. DRV+/-
	pinMode( DRV_2605_EN,	        OUTPUT );	// Output for ERV2605 ENable
	pinMode( PWM_OUT,		OUTPUT );	// Output for PWM
        pinMode( TRIGPIN,               OUTPUT);
        pinMode( ECHOPIN,               INPUT);

	digitalWrite( SW_LRA_M,		LOW );	// Select M+ and M- 
	digitalWrite( SW_MOS_DRV,	HIGH );	// Select +3.3V and MOS- 
	digitalWrite( DRV_2605_EN,	LOW );	// Disable the DRV2605 (low power mode)
	digitalWrite( PWM_OUT,		LOW );	// PWM output low for now 

	pinMode( MOTOR_PIN_0,	OUTPUT );	// Motor select 1
	pinMode( MOTOR_PIN_1,	OUTPUT );	// Motor select 2
}	// setupPins


long measuredistance()
{

 const int pingPin = 12;
 long duration, distance;
  /* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 
 /* set pin mode to output */
  pinMode(pingPin, OUTPUT);

 digitalWrite(pingPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(pingPin, HIGH);
 delayMicroseconds(10); 
 
 analogWrite(pingPin, LOW);

 pinMode(pingPin, INPUT);
 duration = pulseIn(pingPin, HIGH);
 
 //Calculate the distance (in cm)
 distance = duration/ 29 / 2;
 return distance;
}

long getLongDistance()
{
  const int analogPin = 1;
  const int configPin = 6;
  const int digitalPin = 7;
  digitalWrite(configPin, HIGH);
  int digital = digitalRead(digitalPin);
  Serial.print("EZ-digital: ");
  Serial.println(digital);
  int analog = analogRead(analogPin);
  Serial.print("EZ-analog: ");
  Serial.println(analog);
  
}

void printdistance(long distance)
{
// if (distance >= maximumRange || distance <= minimumRange)
// {
//    Serial.println("Error: Out of Range");
// }
// 
// else 
// {
   Serial.print("Distance: ");
   Serial.print(distance);
   Serial.println(" cm");
// }
}

void getdistancegroup(long distance)
{
   if (0 <= distance && distance <= 5)
   {
     distancegroup = 1;
   }
   
  else if (6 <= distance && distance <= 10)
   {
     distancegroup = 2;
   }
   
   else if (11 <= distance && distance <= 15)
   {
     distancegroup = 3;
   }
   
  else if (16 <= distance && distance <= 20)
   {
     distancegroup = 4;
   }

  else if (21 <= distance && distance <= 30)
   {
     distancegroup = 5;
   }
   
  else if (30 <= distance)
   {
     distancegroup = 6;
   }

   Serial.print("Distancegroup: ");
   Serial.println (distancegroup);
}

void playeffect()
{
  switch (distancegroup)
  {
   case 1:
     if (measurecount < 1)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 15);
     Serial.println("Under 5 cm");
     measurecount = 0;
     break;
     
   case 2:
     if (measurecount < 2)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 47);
     Serial.println("Under 10 cm");
     measurecount = 0;
     break;
     
    case 3:
     if (measurecount < 4)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 48);
     Serial.println("Under 15 cm");
     measurecount=0;
     break;
     
     case 4:
     if (measurecount < 4)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 49);
     Serial.println("Under 20 cm");
     measurecount=0;
     break;
     
     case 5:
     if (measurecount < 9)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 50);
     Serial.println("Under 30 cm");
     measurecount=0;
     break;
     
     case 6:
     if (measurecount < 9)
     {
       measurecount++;
       break;
     }
     motor.playFullHaptic(3, 51);
     Serial.println("Over 30 cm");
     measurecount=0;
     break;
 }
}
