// This Software demo for the Precision Microdrives Haptic Shield is provided
// under the MIT License

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to 
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

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
long duration, distance;            // Duration used to calculate distance
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
  Serial.println("Measuring distance");
  measuredistance();
  printdistance();
  Serial.println("Getting distance group");
  getdistancegroup();
  Serial.println("Playing effect");
  playeffect();
 
  delay(100);
  
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


void measuredistance()
{

 const int pingPin = 12;
 
  /* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 
 /* set pin mode to output */
  pinMode(pingPin, OUTPUT);

 digitalWrite(pingPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(pingPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(pingPin, LOW);

 pinMode(pingPin, INPUT);
 duration = pulseIn(pingPin, HIGH);
 
 //Calculate the distance (in cm)
 distance = duration/ 29 / 2;
}

void printdistance()
{
 if (distance >= maximumRange || distance <= minimumRange)
 {
    Serial.println("Error: Out of Range");
 }
 
 else 
 {
   Serial.print("Distance: ");
   Serial.print(distance);
   Serial.println(" cm");
 }
}

void getdistancegroup()
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
