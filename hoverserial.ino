// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// • THIS VERSION USES MARCO'S NUNCHUK ADAPTATION, REMEMBER TO GET THE NINTENDOEXTENSIONCTRL LIBRARY AND CONNECT THE NUNCHUCK TO THE RESPECTIVE I2C PINS ON YOUR BOARD: https://www.arduino.cc/reference/en/language/functions/math/map/
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           50         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
#include <NintendoExtensionCtrl.h>
SoftwareSerial HoverSerial(2,3); 		    // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t	start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t 	cmd1;
   int16_t 	cmd2;
   int16_t 	speedR_meas;
   int16_t 	speedL_meas;
   int16_t 	batVoltage;
   int16_t 	boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ### Joystick
 Nunchuk nchuk;  
 int joyPin1 = 0;                 // slider variable connecetd to analog pin 0
 int joyPin2 = 1;                 // slider variable connecetd to analog pin 1
 int joyVal1 = 0;                  // variable to read the value from the analog pin 0
 int joyVal2 = 0;
 int joyTreatVal = 0; 
 int sp = 0;   
 int mappedx = 0;
 int mappedy = 0;


// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0 Edited by Marco for Fabian");
  nchuk.begin();
    
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}


/*int joyTreatValue(int data) {
  //
 joyTreatVal = map(data, 0, 1023, -1000,1000);
 if (abs(joyTreatVal) >= 999){
  joyTreatVal = 0;
 }
  return joyTreatVal;
//  return (data * 9 / 1024) + 48;
 }*/

 void getJoyVals(){
  /*joyVal1 = joyTreatValue(analogRead(joyPin1))+5;
  delay(10);
  joyVal2 = joyTreatValue(analogRead(joyPin2))-2;
  */
//int spd = 500;
//int str = 500;
  
  
  int joyX = nchuk.joyX();
  int joyY = nchuk.joyY();
  joyVal1 = map(joyX, 26, 221, -500, 500);  //Turning speed
  joyVal2 = map(joyX, 26, 224, -1000, 1000); //Acelarating speed
  delay(13);
  
  if (abs(joyVal2) < 100) {
    joyVal2 = 0;   
  }


  if (abs(joyVal1) < 50) {
    joyVal1 = 0;   
  }

/*  while(abs(spd) >= 500 && abs(spd) <= 1000) {
    spd += 10;
    delay(10);
  }

  if(abs(spd) < 500) {
    spd = 500;
  }
  
    while(abs(str) >= 500) {
    Spd += 10;
  }

  while(abs(Spd) < 500) {
    Spd = 500;
  }
 */ 
 }
 void printJoyVals(){
  Serial.print("Jx: "); Serial.println(joyVal1);
    Serial.print("Jy: "); Serial.println(joyVal2);
 } 


// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
	// Check for new data availability in the Serial buffer
	if (HoverSerial.available()) {
		incomingByte 	  = HoverSerial.read();		                              // Read the incoming byte
		bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	  // Construct the start frame		
	}
	else {
		return;
	}

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
		Serial.print(incomingByte);
		return;
	#endif    	
	
	// Copy received data
	if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
		p 		= (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
		*p++ 	= incomingByte;
		idx 	= 2;	
	} else if (idx >= 2 && idx < sizeof(SerialFeedback)) {	// Save the new received data
		*p++ 	= incomingByte; 
		idx++;
	}	
	
	// Check if we reached the end of the package
	if (idx == sizeof(SerialFeedback)) {  	
		uint16_t checksum;
		checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
					^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
	
		// Check validity of the new data
		if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
			// Copy the new data
			memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
		  
			// Print data to built-in Serial
			Serial.print("1: ");   Serial.print(Feedback.cmd1);
			Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
			Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
			Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
			Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
			Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
			Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
		} else {
		  Serial.println("Non-valid data skipped");
		}
		idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
	}
 	
	// Update previous states
	incomingBytePrev 	= incomingByte;
}

/*void buttons() {

   int F = digitalRead(12);
   
   if(F == 0) {

    sp = 600;
    for(sp = 0; sp <= 999; sp++) {
      delay(10);
    }
   } else {
    sp = 100;
   }  
   Serial.println(F);
} */

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;

void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  getJoyVals();
 // printJoyVals();

  //buttons();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  //Send(0, SPEED_MAX_TEST - 2*abs(iTest));
  
  
Send(joyVal1, joyVal2);
//Send(0,sp);
  // Calculate test command signal
  //iTest += 10;
  //if (iTest > iTestMax) iTest = -iTestMax;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
