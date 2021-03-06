/****************************************************************
   encoder_test-1
   This program uses the teensy encoder library
   http://www.pjrc.com/teensy/td_libs_Encoder.html
   to test the encoder on one motor of the DPRG
   club robot (v2016). 
   Code by Doug Paradis (dep) version: 20160719
   
   Derived from:
   - Teensy Encoder library by pjrc
   - MonsterMoto Shield Example Sketch by Jim Lindblom

   Using the wiring harness that came with the motor used
   on the DPRG club robot the connections are:
   Black - motor shield A2
   Green - motor shield B2
   Red - either 5v or 3.3v
   Orange - ground
   Yellow (encoder ch A) - Arduino Mega pin 21 (INT0)
   White (encoder ch B) - Arduino Mega pin 20 (INT1)
   
   Additional connections:
   Battery Red - motor shield +
   Battery Black - motor shield -
   Power Plug to Arduino Mega is DISCONNECTED (Mega power from
   USB port)
   
   Expected behavior:
   Right motor (looking from rear of robot) should pause, move CW
   10 revolutions, pause, move CCW 10 revolutions,pause, then
   repeat pattern. Use tape on wheel to track wheel movement. 
   In other words, CW turning is defined when looking along the axis of the motor,
   with the body of the motor between you and the enecoder 
   Using the Serial Monitor window of the Arduino environment, when turning CW,
   encoder counts should increase from 0 to positive ~32920, then back down to ~0. 
   Note that counts may not be exact for several reasons. For example, issues with 
   the encoder sensors may lead to missed counts. Also, this program polls the encoder tally.
   At typical motor speeds, this program is not able to read the encoder tally
   as often as it changes.  However, the library which maintains the encoder tally is
   interrupt driven, and should be able to keep up.

   Motor Polarity Problem:
   On some motors, the encoder is mounted 180 degrees opposite of motors assembled with the typical convention.
   This yields the effect that the same sofware commands and wiring harness which normally 
   produce CW rotation actually produce CCW rotation. In such cases, if you are not equipped
   to desolder and rotate the encoder, consider simply flipping
   the motor polarity in software or the wiring harness for the offending motor,
     
   Encoder Problem:
   If the wheel doesn't turn 10 revolutions during the test or the
   encoders counts are way off, you have an encoder issue.
   
    Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) 
    function to get motors going in either CW, CCW, BRAKEVCC, or 
    BRAKEGND. Use motorOff(int motor) to turn a specific motor off.
 
    The motor variable in each function should be either a 0 or a 1.
    pwm in the motorGo function should be a value between 0 and 255.
 *******************************************************************/
   

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(21,20);   // Mega INT0 and INT1
//   avoid using pins with LEDs attached


// Motor shield setup
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 13;      // attached to LED

int oneRev = 3292;   // Full Quadrature encoder cnts for one rev
int flgs = 0;
int reachedTarget = 0;
int rotationDir = CW;
char motorSpd = 20;   //slow enough to count and avoid inertia driven motion
                      //too slow and motor stalls.  
                      //too fast and inertia makes it difficult to count & difficult to see when the motor drive stops

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  Serial.println("Start by turning CW...");
  
  // motor shield pin setup
  pinMode(statpin, OUTPUT);
  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  
  delay(3000);
  
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("-> ");
    Serial.println(newPosition);	
  }

  //motorGo(0, rotationDir, motorSpd);
  motorGo(1, rotationDir, motorSpd);

  //if ((newPosition <= ((long)oneRev * -10)) && (flgs == 0)){    // backwards motor wiring
  if ((newPosition >= ((long)oneRev * 10)) && (flgs == 0)){       // normal motor wiring
   Serial.println("finished turning CW");
   reachedTarget = 1;
  }

  //if ((newPosition >= 0) && (flgs == 1)){  // backwards motor wiring
  if ((newPosition <= 0) && (flgs == 1)){    // normal motor wiring
    reachedTarget = 1;
    Serial.println("finished turning CCW");
  }

  if (reachedTarget == 1){
    //motorOff(0);
    motorOff(1);
    Serial.println(newPosition);
    Serial.println("- reached target -");
    newPosition = 0;
    reachedTarget = 0;
    if (flgs == 0){
      flgs = 1;
      rotationDir = CCW;
      Serial.println("- reached target - now turning CCW");
    }
    else if (flgs == 1){
      flgs = 0;
      rotationDir = CW;
      Serial.println("- reached target - now turning CW");
    }
    delay(5000);
  }
  
    
  
  if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
    digitalWrite(statpin, HIGH);
}

//---------- Functions -------------------------------
void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between 0 and 255, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
 
