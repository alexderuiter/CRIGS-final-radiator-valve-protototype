
/*----------------------------------------------------------------------------------------------------------------------------------------------
   Haptic feedback with a three phase BLDC
   (c) Frank van Valkenhoef 2016
   #confidential#

   !!!!!!!!!!!!!!!
   By using this code you automatically agree to the following confidentiallity agreement between Frank van Valkenhoef, hereinafter referred to as “The Creator” and you,
   hereinafter referred to as “The Receiving Party” for the purpose of preventing the unauthorized disclosure of Confidential Information as defined below.
   The parties agree to enter into a confidential relationship with respect to the disclosure of certain proprietary and confidential information ("Confidential Information").

    1. Definition of Confidential Information.
       For purposes of this Agreement, "Confidential Information" shall include all information or material that has or could have commercial value or other utility in the business in which The Creator is engaged.
       If Confidential Information is in written form, The Creator shall label or stamp the materials with the word "Confidential" or some similar warning.
       If Confidential Information is transmitted orally, The Creator shall promptly provide a writing indicating that such oral communication constituted Confidential Information.

    2. Exclusions from Confidential Information.
       The Receiving Party's obligations under this Agreement do not extend to information that is:
       (a) publicly known at the time of disclosure or subsequently becomes publicly known through no fault of The Receiving Party;
       (b) discovered or created by The Receiving Party before disclosure by The Creator;
       (c) learned by The Receiving Party through legitimate means other than from The Creator or The Creator's representatives; or
       (d) is disclosed by The Receiving Party with The Creator's prior written approval.

    3. Obligations of The Receiving Party.
       The Receiving Party shall hold and maintain the Confidential Information in strictest confidence for the sole and exclusive benefit of The Creator.
       The Receiving Party shall carefully restrict access to Confidential Information to employees, contractors, and third parties as is reasonably required and shall require those persons to sign nondisclosure restrictions at least as protective as those in this Agreement.
       The Receiving Party shall not, without prior written approval of The Creator, use for The Receiving Party's own benefit, publish, copy, or otherwise disclose to others, or permit the use by others for their benefit or to the detriment of The Creator, any Confidential Information. The Receiving Party shall return to The Creator any and all records, notes, and other written, printed, or tangible materials in its possession pertaining to Confidential Information immediately if The Creator requests it in writing. The Receiving Party will credit The Creator in all written, verbal or visual communications to others than The Creator concerning information provided by The Creator or affiliated information.

    4. Time Periods.
       The nondisclosure provisions of this Agreement shall survive the termination of this Agreement and The Receiving Party's duty to hold Confidential Information in confidence shall remain in effect until the Confidential Information no longer qualifies as a trade secret or until The Creator sends The Receiving Party written notice releasing The Receiving Party from this Agreement, whichever occurs first.

    5. Relationships.
       Nothing contained in this Agreement shall be deemed to constitute either party a partner, joint venturer or employee of the other party for any purpose.

    6. Severability.
       If a court finds any provision of this Agreement invalid or unenforceable, the remainder of this Agreement shall be interpreted so as best to effect the intent of the parties.

    7. Integration.
       This Agreement expresses the complete understanding of the parties with respect to the subject matter and supersedes all prior proposals, agreements, representations, and understandings.
       This Agreement may not be amended except in a writing signed by both parties.

    8. Waiver.
       The failure to exercise any right provided in this Agreement shall not be a waiver of prior or subsequent rights.
  ----------------------------------------------------------------------------------------------------------------------------------------------*/


//setmotorDriverpins
#define IN1 6
#define IN2 7
#define IN3 8
#define EN  9

//rotary encoder
#define SELECT_PIN 10
#define DATA_PIN   11
#define CLOCK_PIN  12
#define LENGTH_ROTARY_ENCODER_DATA 16
volatile int value = 0;
volatile int absoluteValue = 0;
volatile byte REcounter = 0;
volatile int previousValue[] = {0, 0, 0, 0, 0};
volatile float averageValue = 0;
volatile float speedFactor = 0;
volatile float angle = 0;
volatile int RERotations = 1;
volatile float absoluteAngle = 0;
float previousAngleSend = 0;
volatile float previousAngle = 0;
volatile boolean clockwise = true;
volatile float hardStopCounter = 0;

//Sine Wave Look up table
const byte pwmSin[] = {127, 130, 133, 136, 139, 142, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185, 187, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 253, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 218, 216, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 187, 185, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 69, 67, 64, 61, 58, 56, 53, 51, 48, 46, 43, 41, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 19, 17, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 41, 43, 46, 48, 51, 53, 56, 58, 61, 64, 67, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185, 187, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 253, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 218, 216, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 187, 185, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 69, 67, 64, 61, 58, 56, 53, 51, 48, 46, 43, 41, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 19, 17, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 41, 43, 46, 48, 51, 53, 56, 58, 61, 64, 67, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 112, 115, 118, 121, 124, 127, 130, 133};
#define phaseLength 252
#define phaseOffset 84
byte mappedSinPWM[360];
volatile float force = 0;
volatile float goalForce = 0;

//motor control variables
volatile int currentStepA;
volatile int currentStepB;
volatile int currentStepC;
volatile float percentage = 1;
volatile int motorDirection = -1;

//data
byte buffer[6];
byte LUTforce[360];
byte LUTclockwise[360];
byte byteCounter = 0;


/*----------------------------------------------------------------------------------------------------------------------------------------------
   Alex' variables
  ----------------------------------------------------------------------------------------------------------------------------------------------*/


//motor
int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int valveTemperature;
float decimalValveTemperature;
float desiredTemperature;
float boundaryValue = 5; // how much may the angle be off when setting it beck to previouse desired angle on both ends (so 30 + 3 or - 3, for instance)
boolean valveTouched = false;
boolean buttonPressed = false;
float setAngle = 0;

////led ring
#include <Adafruit_NeoPixel.h>

#define PIN 3

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
    


int whiteValue = 0;
int redValue = 0;
int greenValue = 0;
int blueValue = 0;







/*----------------------------------------------------------------------------------------------------------------------------------------------
   initialize Timer interrupt function
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency;                                                    // 128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2);                                                                  // 50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}







/*----------------------------------------------------------------------------------------------------------------------------------------------
   SetUp function
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {
  //delay to prevent serial prints the first seconds so they do not interfere with uploading new code
  delay(2500);


  //input pins for rotary encoder
  pinMode(SELECT_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  SerialUSB.begin(115200);
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // Mapping Sine Wave Loop up table positions to rotary encoder positions
  calibrate();

  // Start timer.
  startTimer(TC1, 0, TC3_IRQn, 200); //last value represents the FPS

  for (int i = 0; i < 360; i++) {
    LUTforce[i] = 1;      // array om te zeggen dat 0-360 geen kracht heeft
    LUTclockwise[i] = 0;
  }


  /*----------------------------------------------------------------------------------------------------------------------------------------------
     Alex' Addition to SetUp function
    ----------------------------------------------------------------------------------------------------------------------------------------------*/
  // led ring setup
//  strip.begin();
//  strip.show(); // Initialize all pixels to 'off'


}







/*----------------------------------------------------------------------------------------------------------------------------------------------
   The Main Loop
   Insert your own code to do other things here
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() {

  
//  while(Serial.read() != 'A') {}
//  while(!Serial.available()) {}
//  char valve = Serial.read();
//  
//  while (Serial.read() != '$') {}
//  while(!Serial.available()) {}
//  char brightness = Serial.read();


  // temperature mapping
  valveTemperature = map(absoluteAngle, 60, 120, 180, 250);
  decimalValveTemperature = valveTemperature / 10;

  // touch sensor
  readFsr();


  // led lights
//  for (int i = 0; i < 16; i++) {
//    strip.setPixelColor(i, redValue, greenValue, blueValue, whiteValue);
//  }
//
//  strip.show();



  //debugging
  Serial.print("speedFactor = ");
  Serial.print(speedFactor);
  Serial.print("\t");
  Serial.print("absoluteAngle = ");
  Serial.print(absoluteAngle);
  Serial.print("\t");
  Serial.print("decimalValveTemperature = ");
  Serial.print(decimalValveTemperature);
  Serial.print("\t");
  Serial.print("desiredTemperature = ");
  Serial.print(decimalValveTemperature);
  Serial.print("\t");
  Serial.print("desiredAngle = ");
  Serial.print(setAngle);
  Serial.print("\t");
  Serial.print("valveTouched = ");
  Serial.println(valveTouched);
  delay(5);
}

/*----------------------------------------------------------------------------------------------------------------------------------------------
   Alex' functions
  ----------------------------------------------------------------------------------------------------------------------------------------------*/

void readFsr() {
  fsrReading = analogRead(fsrPin);
  if (fsrReading > 5) {
    valveTouched = true;
  } else {
    valveTouched = false;
  }
  if (fsrReading > 1000) {
    buttonPressed = true;
//    redValue = 225;
  } else {
    buttonPressed = false;
  }
}




/*----------------------------------------------------------------------------------------------------------------------------------------------
   Timer 3 Interrupt function
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void TC3_Handler()
{ // You must do TC_GetStatus to "accept" interrupt
  // As parameters use the first two parameters used in startTimer (TC1, 0 in this case)
  TC_GetStatus(TC1, 0);

  readRotary();
  int aA = int(absoluteAngle);
  int offset;
  boolean forceClockwise = true;


  //Program your effects here---------------------------------------------------------------------------------

  if (!valveTouched) {
    if (absoluteAngle < setAngle - boundaryValue) {
      forceClockwise = true;
      force = (setAngle - absoluteAngle) * 0.025;
      //      if (setAngle - absoluteAngle < (10*PI)){                                      <<< why does this not work??
      //      force = 0.5 * sin(((setAngle - absoluteAngle)/10)+(0.5*PI)) + 0.5 ;
      //      } else {
      //       force = 1;
      //      }
    }
    else if (absoluteAngle > setAngle + boundaryValue) {
      forceClockwise = false;
      force = (absoluteAngle - setAngle) * 0.025;
    }
    else {
      force = 0;
    }
  } else if (valveTouched) {
    if (clockwise) {
      forceClockwise = false;
    }
    else if (!clockwise) {
      forceClockwise = true;
    }
    force = abs(speedFactor) / 120; //adding resistance depending if movement is detected // 120 was used often
    if (buttonPressed) {
      setAngle = absoluteAngle;
      desiredTemperature = decimalValveTemperature;
    }
  }

  if (force > 1) {
    force = 1; // force may never be bigger than 1
  }



  /*--------------------------------------------------------------------------------------------------------*/


  if (forceClockwise == true) {
    offset = 10;
  } else {
    offset = 350;
  }

  currentStepA = mappedSinPWM[(aA + offset) % 360];
  currentStepB = currentStepA + phaseOffset;
  currentStepC = currentStepB + phaseOffset;

  analogWrite(IN1, pwmSin[currentStepA]*force);
  analogWrite(IN2, pwmSin[currentStepB]*force);
  analogWrite(IN3, pwmSin[currentStepC]*force);
}







/*----------------------------------------------------------------------------------------------------------------------------------------------
   Calibrate function
   Mapping Sine Wave Look up table positions to rotary encoder positions
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void calibrate() {
  moveMotor();
  delay(5);
  readRotary();
  float startPosition = absoluteAngle;              //take starting position

  Serial.print("startPos = "); Serial.println(startPosition);



  for (int i = 0; i < 500; i++) {                    //take a few steps
    moveMotor();
    delayMicroseconds(1000);
  }

  readRotary();
  float currentPosition = absoluteAngle;            //take new position to compare

  Serial.print("secondPos = "); Serial.println(currentPosition);

  // reverse direction of the motor if it is going counter clockwise
  if (currentPosition < startPosition) {
    motorDirection = -1;
    Serial.println("switch direction");
  }

  //rotate 360º and store value of currentStepA (the position of channel A in the sinewave) in an array
  float calibrationStartPosition = absoluteAngle + 1;
  for (int i = 0; i < 360; i++) {
    float goalPosition = calibrationStartPosition + i;
    readRotary();
    while (absoluteAngle < goalPosition - 0.5) {
      moveMotor();
      delay(5);
      readRotary();
      absoluteAngle;
    }
    mappedSinPWM[int(goalPosition) % 360] = currentStepA;
  }

  Serial.println("DONE CALIBRATING!!!!!!!!!!!!!!!!!!!");


  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
}







/*----------------------------------------------------------------------------------------------------------------------------------------------
   MoveMotor function
   Move the motor by stepping through the SineWave array
   This is used to move the motor during calibration
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void moveMotor() {
  currentStepA += motorDirection;

  //Check for lookup table overflow and return to opposite end if necessary
  if (currentStepA > phaseLength)  currentStepA = 0;
  else if (currentStepA < 0)  currentStepA = phaseLength;

  currentStepB = currentStepA + phaseOffset;
  currentStepC = currentStepB + phaseOffset;

  analogWrite(IN1, pwmSin[currentStepA] * percentage);
  analogWrite(IN2, pwmSin[currentStepB] * percentage);
  analogWrite(IN3, pwmSin[currentStepC] * percentage);
}









/*----------------------------------------------------------------------------------------------------------------------------------------------
   Rotary encoder functions
  ----------------------------------------------------------------------------------------------------------------------------------------------*/
void readRotary() {
  value = 0;

  //Write selectpin low to enable data reception
  digitalWrite(SELECT_PIN, LOW);

  //for (int i = LENGTH_ROTARY_ENCODER_DATA-3; i >= 0 ; i--)
  for (int i = 11; i > 0 ; i--)
  {
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);

    uint8_t bit = digitalRead(DATA_PIN);
    value |= bit << i;
  }

  // Write select pin high to disable data reception
  digitalWrite(SELECT_PIN, HIGH);

  previousAngle = absoluteAngle;
  angle = float(value) / 4096 * 360;    //the rotary encoder has 4096 PPR
  absoluteAngle = angle + RERotations * 360;
  absoluteValue = value + RERotations * 4096;

  //check for passing the index point and compensate with an increment or decrement of the number of rotations
  if (absoluteAngle < previousAngle - 300) {
    RERotations ++;
    absoluteAngle += 360;
    absoluteValue += 4096;
  } else if (absoluteAngle > previousAngle + 300) {
    RERotations --;
    absoluteAngle -= 360;
    absoluteValue -= 4096;
  }

  //calculate the average position over the previous 5 measurements
  averageValue = (previousValue[0] + previousValue[1] + previousValue[2] + previousValue[3] + previousValue[4]) / 5;

  //calculate the speedfactor
  speedFactor = absoluteValue - averageValue;

  //check direction
  if (absoluteAngle > previousAngle) {
    clockwise = true;
  } else {
    clockwise = false;
  }

  previousValue[REcounter] = absoluteValue;

  REcounter++;
  if (REcounter == 5) {
    REcounter = 0; //check for out of bound of previousValue[]
  }
}

