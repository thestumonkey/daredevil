#include "motor.h"
#include "DRV2605.h"
#include "Metro.h"
#include <EEPROM.h>

Motor motor = Motor();

bool playing = false;
bool playingVIB = false;
bool playingAudio = false;

DRV2605 drv2605 = DRV2605();
static uint8_t motorBitmask;
MotorInfo keyMotor, currentMotor;

Motor::Motor()
:	stopPlaying( false )
,	selectedMotor( 0 )
{
	// Set initial motor
	Serial.println( F("Resetting" ));
	readMotorInfo( selectedMotor, &currentMotor );
}

void Motor::init( uint8_t keyMotorId )
{
	// Initialise the DRV
	drv2605.init();

	readMotorCals();
	setupKeyMotor( keyMotorId );
	selectMotor( 0 );
} // init

void Motor::readMotorCals()
{
	// Motor calibration bitmask is stored inverted as EEPROM bytes are initialised to 0xFF
	// If it wasn't inverted, we would start thinking all motors are calibrated
	motorBitmask = ~EEPROM.read( MOTOR_VALS_ADDR );
	Serial.print( F("Saved cals bitmask: " ));
	Serial.println( motorBitmask, BIN );
} // readMotorCals

void Motor::resetAutoCal()
{
	motorBitmask = 0x00;
	EEPROM.write( MOTOR_VALS_ADDR, ~motorBitmask );

	for( uint8_t addr = 0x00 ; addr < MOTOR_AC_LEN ; ++addr )
	{
		EEPROM.write( MOTOR_AC_ADDR + addr, 0xFF );
	}
} // resetAutoCal

void Motor::setupKeyMotor( uint8_t id )
{
	readMotorInfo( id, &keyMotor );
	
	Serial.println(id);

	if( !isMotorCaled( id ) )
	{
		Serial.println(F( "Calibrating key motor" ));
		calibrate( &keyMotor, true );
	}
	else
	{
		Serial.println( F("Key motor already caled" ));
		playKeyPress( false );
	}
} // setupKeyMotor

void Motor::readMotorInfo( uint8_t id, MotorInfo* info )
{
	info->id = pgm_read_byte( &motorTable[ id ].id );
	info->LRA = pgm_read_byte( &motorTable[ id ].LRA );
	info->min_duty = pgm_read_byte( &motorTable[ id ].min_duty );
	info->rated_duty = pgm_read_byte( &motorTable[ id ].rated_duty );
	info->max_duty = pgm_read_byte( &motorTable[ id ].max_duty );
	info->LRA = pgm_read_byte( &motorTable[ id ].LRA );
	
	for( uint8_t i = 0 ; i < 7 ; ++i ) 
	{ // strncpy_P() doesn't like a volatile target, so do it like this:
		info->part_num[ i ] = pgm_read_byte( &motorTable[ id ].part_num[ i ] );
	}
	
	info->part_num[ 7 ] = 0; // Be sure zero terminated.
} // readMotorInfo

bool Motor::isMotorCaled( uint8_t id )
{
	return motorBitmask & ( 1 << id );
} // isMotorCaled

void Motor::setMotorCal( uint8_t id, uint8_t compensation, uint8_t backEMF, uint8_t feedback )
{
	motorBitmask |= ( 1 << id );
	EEPROM.write( MOTOR_VALS_ADDR, ~motorBitmask );

	// Find the correct EEPROM address from the id
	id *= MOTOR_AC_SIZE;
	id += MOTOR_AC_ADDR;

	EEPROM.write( id + 0, compensation );
	EEPROM.write( id + 1, backEMF );
	EEPROM.write( id + 2, feedback );

	Serial.print( F("Saving auto cals: 0x") );
	Serial.print( compensation, HEX );
	Serial.print( F(" 0x") );
	Serial.print( backEMF, HEX );
	Serial.print( F(" 0x") );
	Serial.println( feedback, HEX );
} // setMotorCal

void Motor::getMotorCal( uint8_t id, uint8_t* compensation, uint8_t* backEMF, uint8_t* feedback )
{
	// Find the correct EEPROM address from the id
	id *= MOTOR_AC_SIZE;
	id += MOTOR_AC_ADDR;

	compensation[ 0 ] =  EEPROM.read( id + 0 );
	backEMF[ 0 ] =  EEPROM.read( id + 1 );
	feedback[ 0 ] =  EEPROM.read( id + 2 );
} // getMotorCal

uint8_t Motor::getMotorId()
{
	return selectedMotor;
} // getMotorId

uint8_t Motor::getKeyMotorId()        //
{
  if (selectedKeyMotor == 0) selectedKeyMotor = 2;
  return selectedKeyMotor;      // LRA =2 ERM=3 due to motor strings order
} // getMotorId



const char* Motor::getMotorName()
{
	return currentMotor.part_num;
} // getMotorName




void Motor::stop()
{
	Serial.println(F( "Stopping" ));
	stopPlaying = true;
} // stop

void Motor::selectMotor( uint8_t motorId )
{
	playKeyPress( false );

	selectedMotor = motorId;
	readMotorInfo( selectedMotor, &currentMotor );

	Serial.print( F("Selecting grip motor " ));
	Serial.println( currentMotor.part_num );

	selectGripMotor( motorId );

	if( !isMotorCaled( selectedMotor ) )
	{
		calibrate( &currentMotor, false );
	}
} // selectMotor

bool Motor::isPlaying()
{
	return playing;
} // isPlaying

bool Motor::isPlayingVIB()
{
	return playingVIB;
} // isPlayingVIB

bool Motor::isPlayingAudio()
{
	return playingAudio;
} // isPlayingVIB


void Motor::playVibAlert( uint8_t waveform, uint8_t pwr, uint8_t onTime, uint8_t offTime )
{
	if( isPlaying() )
		return;
	
	if( currentMotor.LRA )
	{
		motor.playKeyPress( true );      // play short click on LRA  
		Serial.println( F("No Vib Alerts on LRA motor" ));
		return;
	}
	
	motor.playKeyPress( false );      // play short click on LRA  

	Serial.println( F("Playing Vib Alert" ));
        
	stopPlaying = false;
	playing = true;
	playingVIB = true;
	uint8_t on = true;
	uint8_t lTime = onTime;      // contains time in ms * 100
        
	uint8_t pwr_range = currentMotor.rated_duty - currentMotor.min_duty;
	uint16_t currentPWR;
	
	// Setup PWM for 31.25kHz and specified % duty.
	TCCR1A |= (1<<COM1A1) | (1<<WGM10);   // 8 bit fast  PWM, not inverted, output on OC1A (D9) A.K.A. PWM_OUT
	TCCR1B |= (1<<WGM12) | (1<<CS10);    //  8 bit fast PWM, prescaler /1
	OCR1A = 0;

	long startTime = millis();

	while( !stopPlaying )
	{
		int t = (millis() - startTime) / lTime ;  
                          
		if( on )
		{
			currentPWR = ( (uint16_t) pwr * pwr_range ) / ( uint16_t ) 100;  // pwr is 0 to 100
			currentPWR = (uint16_t) (currentPWR * calculatePWR( waveform, t )) / ( uint16_t ) 100;  // calculate power returns 0 - 100
			currentPWR += currentMotor.min_duty;
			OCR1A = currentPWR;	// OCR1 is 0-255 
          
			if( t >= 100 )		// 30ms * 100( min time period)
			{
			
				if( offTime > 0 )
				{
					lTime = offTime;
					on = false;
					OCR1A = 0;
				}
				
				startTime = millis();
			}
		}
		else if( t >= 100 )        // if on = false and t=> 1 now shoulb be on immediatelly
		{
			startTime = millis();
			lTime = onTime;
			on = true;
		}
	}//while
	
	TCCR1A = 0x00; // No PWM
	TCCR1B = 0x00; // stop timer 1
	OCR1A = 0;
	digitalWrite( PWM_OUT, LOW ); // PWM output low until further notice.
	playingVIB = false; 
	playing = false;
} // playVibAlert

int Motor::calculatePWR( uint8_t waveform, int t )
{
	if( t < 0 )  t = 0;
	else if( t > 100 ) t = 100;

	switch( waveform )
	{
		// Square
		default:
		case 0:
			t = 100;  // not this is divider in 
			break;

		// Sine
		case 1:	
			t /= 5;		// reduce range to <0 , 20>
			t -= 10;	// move range from <0, 20> to <-10 ,10>
			t = 10 - (abs( t ));	// now range is 0-10-0 
			t = pgm_read_byte( &halfsineTable[ t ]);
			break;

		// Triangle
		case 2:
			t -= 50;    // move range from <0, 100> to <-50 ,50>
			t = 100 - (abs( t ) * 2 );
			break;

		// Sawtooth
		case 3:
			// return t without changes          
			break;
	}

	return t;  
} // calculatePWR

void Motor::playFullHaptic( uint8_t library, uint8_t effect, bool withKeyPress )
{
	if( isPlaying() )
		return;

	if( withKeyPress )
		playKeyPress( false );


//	digitalWrite( SW_LRA_M,		LOW );	// Select the grip
//	digitalWrite( SW_MOS_DRV,	LOW );	// Select DRV+ and DRV- 
        
        digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
	digitalWrite( GRIP_SEL,	        LOW );	        // GRIP connected
	digitalWrite( SRC_SEL,	        HIGH );	        // DRV connected

	digitalWrite( DRV_2605_EN,	HIGH );	// Enable the DRV2605

	if( !isMotorCaled( currentMotor.id ) )
	{
		calibrate( &currentMotor, false );
	}

	uint8_t compensation, backEMF, feedback;
	getMotorCal( selectedMotor, &compensation, &backEMF, &feedback );

	if( currentMotor.LRA )
	{
		library = 6;
	}
	else
	{
		if( library < 1 )
			library = 1;
		else if( library > 5 )
			library = 5;
	}

	if( effect < 1 )  effect = 1;
	else if( effect > 124 )	effect = 124;

	drv2605.playFullHaptic( library, effect, currentMotor.rated_duty, currentMotor.max_duty, compensation, backEMF, feedback );

	digitalWrite( DRV_2605_EN,	LOW );	// Disable the DRV2605 (low power mode)
//	digitalWrite( SW_MOS_DRV,	HIGH );	// Select +3.3V and MOS- 
//	digitalWrite( SW_LRA_M,		LOW );	// Select M+ and M-

        digitalWrite( LRA_SEL,	        HIGH );	        // SelectLRA
	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
	digitalWrite( GRIP_SEL,	        LOW );	        // GRIP disconnected
	digitalWrite( SRC_SEL,	        LOW );	        // DRV disconnected


	playing = false;
} // playFullHaptic


//void Motor::Audio2Haptic( uint8_t library, uint8_t effect, bool withKeyPress )

void Motor::Audio2Haptic(  bool withKeyPress )
{

  uint8_t lra_Audio;

  if( withKeyPress ) playKeyPress( false );   //short click on LRA

	if( isPlaying() || motor.isPlayingAudio()) return;  //if already playing audio - return

  stopPlaying = false;
	playingAudio = true;

	pinMode( PWM_OUT,	INPUT );	              // Input for PWM
  digitalWrite( PWM_OUT,		LOW );	        // pull up
        
  digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
	digitalWrite( GRIP_SEL,	        LOW );	        // GRIP connected
	digitalWrite( SRC_SEL,	        HIGH );	        // DRV connected

	digitalWrite( DRV_2605_EN,	HIGH );	// Enable the DRV2605

	if( !isMotorCaled( currentMotor.id ) )	calibrate( &currentMotor, false );
	
	uint8_t compensation, backEMF, feedback;
	
	getMotorCal( selectedMotor, &compensation, &backEMF, &feedback );


 if( motor.getMotorId()==3) lra_Audio = 1;
 else lra_Audio = 0;


	drv2605.Audio( lra_Audio, currentMotor.rated_duty, currentMotor.max_duty, compensation, backEMF );

         Serial.println( F("Playing Audio" ));
         
         while( !stopPlaying ){ digitalWrite( PWM_OUT,		HIGH );	        // PWM output low for now 
          };
         
         Serial.println( F("No Audio" ));
                                 
    
        drv2605.setDefaults();

        digitalWrite( DRV_2605_EN,	LOW );	// Disable the DRV2605 (low power mode)
      	
        digitalWrite( SRC_SEL,	        LOW );	        // MOSFET as source
      	digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
      	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
      	digitalWrite( GRIP_SEL,	        HIGH );	        // GRIP connected
      
      	pinMode( PWM_OUT,	OUTPUT );	// Input for PWM
      	digitalWrite( PWM_OUT, LOW ); // PWM output low until further notice.

        playingAudio = false;
       
} // Audio2Haptic

/*
void Motor::StopAudio2Haptic(){
    
}
*/

void Motor::selectGripMotor( uint8_t id )
{
	digitalWrite( GR_SEL1,	id % 2 == 0 ? LOW : HIGH );
	digitalWrite( GR_SEL2,	id > 1 ? HIGH : LOW );
} // selectGripMotor

void Motor::calibrate( MotorInfo* info, boolean shield )
{
//	digitalWrite( SW_LRA_M,		shield ? HIGH : LOW );	// Select the LRA on the shield or M+ and M- to connect the grip.
//	digitalWrite( SW_MOS_DRV,	LOW );	// Select DRV+ and DRV- 

        if (shield){
                digitalWrite( LRA_SEL,	        LOW );	        // SelectLRA
        	digitalWrite( GRIP_SEL,	        HIGH );	        // GRIP disconnected

        }else{
                digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	        digitalWrite( GRIP_SEL,	        LOW );	        // GRIP connected
       }

	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
        digitalWrite( SRC_SEL,	        HIGH );	        // DRV connected
	digitalWrite( DRV_2605_EN,	HIGH );	// Enable the DRV2605

	if( !shield )
	{
		// Select motor on grip
		selectGripMotor( info->id );
	}

	// Auto calibrate the motor
	uint8_t compensation, backEMF, feedback;

	if( drv2605.autoCal( 
			info->rated_duty,
			info->max_duty,
			info->LRA,
			&compensation, &backEMF, &feedback ) )
	{
		Serial.println( F("AutoCal success" ));
		setMotorCal( info->id, compensation, backEMF, feedback );
	}
	else
	{
		Serial.println( F("AutoCal failed" ));
	}

        digitalWrite( DRV_2605_EN,	LOW );	// Disable the DRV2605 (low power mode)

	digitalWrite( SRC_SEL,	        LOW );	        // MOSFET as source
	digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
	digitalWrite( GRIP_SEL,	        HIGH );	        // GRIP connected



//	digitalWrite( SW_MOS_DRV,	HIGH );	// Select +3.3V and MOS- 
//	digitalWrite( SW_LRA_M,		LOW );	// Select M+ and M-
} // calibrate

void Motor::playKeyPress( bool longPress )
{
	playing = true;

  	if( !isMotorCaled( keyMotor.id ) )
	{
		calibrate( &keyMotor, true );
	}

//	digitalWrite( SW_LRA_M,		HIGH );	// Select the LRA on the shield
//	digitalWrite( SW_MOS_DRV,	LOW );	// Select DRV+ and DRV- 

	digitalWrite( GRIP_SEL,	        HIGH );	        // GRIP disconnected

        if(keyMotor.id == KEY_MOTOR_LRA){

            selectedKeyMotor = KEY_MOTOR_LRA;
            digitalWrite( LRA_SEL,	        LOW );	        // LRA connected
	          digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
        
        }else if(keyMotor.id == KEY_MOTOR_ERM){

            selectedKeyMotor = KEY_MOTOR_ERM;
            digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	          digitalWrite( ERM_SEL,	        LOW );	        // ERM connected
        }
	digitalWrite( SRC_SEL,	        HIGH );	        // DRV as source
	digitalWrite( DRV_2605_EN,	HIGH );	// Enable the DRV2605

	uint8_t compensation, backEMF, feedback;
	getMotorCal( keyMotor.id, &compensation, &backEMF, &feedback );
	
        if(keyMotor.id == KEY_MOTOR_LRA){
              drv2605.playFullHaptic( 6, longPress ? LONG_PRESS_EFF : SHORT_PRESS_EFF,          //FOR ERM 
	                        keyMotor.rated_duty, keyMotor.max_duty,
	                        compensation, backEMF, feedback );
        }else if(keyMotor.id == KEY_MOTOR_ERM){
        
              drv2605.playFullHaptic( 1, longPress ? LONG_PRESS_EFF : SHORT_PRESS_EFF,          //FOR ERM 
	                        keyMotor.rated_duty, keyMotor.max_duty,
	                        compensation, backEMF, feedback );
        }
	digitalWrite( DRV_2605_EN,	LOW );	// Disable the DRV2605 (low power mode)

	digitalWrite( LRA_SEL,	        HIGH );	        // LRA disconnected
	digitalWrite( ERM_SEL,	        HIGH );	        // ERM disconnected
	digitalWrite( GRIP_SEL,	        LOW );	        // GRIP connected
	digitalWrite( SRC_SEL,	        LOW );	        // MOSFET as source


//	digitalWrite( SW_MOS_DRV,	HIGH );	// Select +3.3V and MOS- 
//	digitalWrite( SW_LRA_M,		LOW );	// Select M+ and M-

	playing = false;
} // playKeyPress
