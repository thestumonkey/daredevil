
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
#include "defs.h"

// Init timer for 2s, and auto-reset when we get a positive check
Metro timer( 2000, 1 );
Motor motorz = Motor();

int maximumRange = 150;             // Maximum range needed
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
	motor.init( KEY_MOTOR_LRA);

	// Ensure any time for calibration is ignored.
	timer.reset();
}

void loop()
{
  long distance;
  int distancegroup;
//  Serial.println("-Measuring distance");
  distance = measuredistance();
//  distance = getLongDistance();
  printdistance(distance);
//  distancegroup = getdistancegroup(distance);
  distancegroup = getLongDistanceGroup(distance);
//distancegroup = getMediumDistanceGroup(distance);
  playeffect(distancegroup);
//  long long_distance = getLongDistance();
 
  delay(10);
  
}

void setupPins()
{
	pinMode( LRA_SEL,  OUTPUT ); // Output for LRA  (0 = connected)
  pinMode( ERM_SEL, OUTPUT ); // Output for ERM  (0 = connected)
  pinMode( GRIP_SEL,  OUTPUT ); // Output for GRIP (0 = connected)
  pinMode( SRC_SEL, OUTPUT ); // Output for switching DRV(1) or MOSFET(0)
  pinMode( DRV_2605_EN, OUTPUT ); // Output for ERV2605 ENable
  pinMode( PWM_OUT, OUTPUT ); // Output for PWM
  pinMode( GR_SEL1, OUTPUT ); // Motor select 1
  pinMode( GR_SEL2, OUTPUT ); // Motor select 2

  pinMode( IRQ_PIN, INPUT );

  pinMode( TRIGPIN,               OUTPUT);

  digitalWrite( LRA_SEL,          HIGH );         // SelectLRA
  digitalWrite( ERM_SEL,          HIGH );         // ERM disconnected
  digitalWrite( GRIP_SEL,         HIGH );         // GRIP disconnected
  digitalWrite( SRC_SEL,          HIGH );         // DRV connected
  digitalWrite( DRV_2605_EN,  LOW );          // Disable the DRV2605 (low power mode)
  digitalWrite( PWM_OUT,    LOW );          // PWM output low for now 
  

  digitalWrite( GR_SEL1,  LOW );          // select motor 0
  digitalWrite( GR_SEL2,  LOW );          //  

  digitalWrite( IRQ_PIN, HIGH );
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

void read_analog_sensor () 
{
  const int anPin = 2;
  pinMode(anPin, INPUT);
  long anVolt = analogRead(anPin);
  Serial.print("volt: ");
  Serial.println(anVolt*5);
}

long getLongDistance()
{
  Serial.print("digital: ");
  const int pwPin1 = 11;
  pinMode(pwPin1, INPUT);
  long sensor = pulseIn(pwPin1, HIGH);
  long distance = sensor/ 29 / 2;
  return distance;
//  Serial.println(distance);
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

int getdistancegroup(long distance)
{
   if (0 <= distance && distance <= 20)
   {
     distancegroup = 1;
   }
   
  else if (20 <= distance && distance <= 45)
   {
     distancegroup = 2;
   }
   
   else if (45 <= distance && distance <= 65)
   {
     distancegroup = 3;
   }
   
  else if (65 <= distance && distance <= 90)
   {
     distancegroup = 4;
   }

  else if (90 <= distance && distance <= 110)
   {
     distancegroup = 5;
   }
   
  else if (110 <= distance && distance <= 130)
   {
     distancegroup = 6;
   }
  else if (130 <= distance)
   {
     distancegroup = 0;
   }
    return distancegroup;
//   Serial.print("Distancegroup: ");
//   Serial.println (distancegroup);
}

int getLongDistanceGroup(long distance)
{
   if (0 <= distance && distance <= 100)
   {
     distancegroup = 1;
   }
   
  else if (100 <= distance && distance <= 150)
   {
     distancegroup = 2;
   }
   
   else if (150 <= distance && distance <= 200)
   {
     distancegroup = 3;
   }
   
  else if (200 <= distance && distance <= 250)
   {
     distancegroup = 4;
   }

  else if (250 <= distance && distance <= 300)
   {
     distancegroup = 5;
   }
   
//  else if (250 <= distance && distance <= 300)
//   {
//     distancegroup = 6;
//   }
  else if (130 <= distance)
   {
     distancegroup = 0;
   }
   Serial.print("Distancegroup: ");
   Serial.println (distancegroup);
  return distancegroup;

}

int getMediumDistanceGroup(long distance)
{
   if (0 <= distance && distance <= 100)
   {
     distancegroup = 1;
   }
   
  else if (100 <= distance && distance <= 150)
   {
     distancegroup = 2;
   }
   
   else if (150 <= distance && distance <= 200)
   {
     distancegroup = 3;
   }
   
  else if (200 <= distance && distance <= 250)
   {
     distancegroup = 4;
   }

  else if (250 <= distance && distance <= 300)
   {
     distancegroup = 5;
   }
   
  else if (250 <= distance && distance <= 300)
   {
     distancegroup = 6;
   }
  else if (300 <= distance)
   {
     distancegroup = 0;
   }
   Serial.print("Distancegroup: ");
   Serial.println (distancegroup);
  return distancegroup;

}

void playeffect(int distancegroup)
{
  const int hap_1 = 15;
  const int hap_2 = 14;
  const int hap_3 = 13;
  const int hap_4 = 52;
  const int hap_5 = 53;
  const int hap_6 = 55;
  const int motor_hap = 1;
  switch (distancegroup)
  {
   case 1:
     if (measurecount <= 1)
     {
       measurecount++;
       break;
     }
     Serial.println("playing1");
     motorz.playFullHaptic(motor_hap, hap_1);
     measurecount = 0;
     break;
     
   case 2:
     if (measurecount <= 1 )
     {
       measurecount++;
       break;
     }
     Serial.println("playing2");
     motorz.playFullHaptic(motor_hap, hap_2);
     measurecount = 0;
     break;
     
    case 3:
     if (measurecount <= 1)
     {
       measurecount++;
       break;
     }
     Serial.println("playing3");
     motorz.playFullHaptic(motor_hap, hap_3);
     measurecount=0;
     break;
     
     case 4:
     if (measurecount <= 1)
     {
       measurecount++;
       break;
     }
     Serial.println("playing4");
     motorz.playFullHaptic(motor_hap, hap_4);
     measurecount=0;
     break;
     
     case 5:
     if (measurecount <= 1)
     {
       measurecount++;
       break;
     }
     Serial.println("playing5");
     motorz.playFullHaptic(motor_hap, hap_5);
     measurecount=0;
     break;
     
     case 6:
     if (measurecount <= 1)
     {
       measurecount++;
       break;
     }
     Serial.println("playing6");
     motorz.playFullHaptic(motor_hap, hap_6);
     measurecount=0;
     break;
 }
}

