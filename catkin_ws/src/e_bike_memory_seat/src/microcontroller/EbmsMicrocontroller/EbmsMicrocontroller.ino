#include <SPI.h>
#include "mcp_can.h"

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
#endif

#define MOTOR_F 5
#define MOTOR_B 6
#define FEEDBACK 2
#define MAX_WORK_TIME 60000
#define STALL_TIME 1000 // TODO should be 26ms. Anything else is for testing purposes.
#define ROS_CAN_ID 0x555


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup() {
    SERIAL.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println("Init CAN BUS Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");

    // initialise motor output pins and set to low.
    pinMode(MOTOR_F, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(FEEDBACK, INPUT);
    digitalWrite(MOTOR_F, LOW);
    digitalWrite(MOTOR_B, LOW);

    Serial.println("Please enter new goalHeight");
}


void loop() {
    readFromCanBus();
    //readFromSerial();
}

/* 
   *  return the current height in mm instead of bytes
   *  1023b / 150 = 6.82
   *  1b = 0.147mm
   *  1mm = 6.82b
   *  when casting to byte the value is truncated not rounded
   *  so 100.1 and 100.9 are both cast to 100
   */
byte getCurrentHeight() {
  return (byte)(analogRead(FEEDBACK) / 6.82);
}

/**
 * There are two safety mechanisms implemented in the raise function.
 * 1.) Overheat protection by limiting the maximum work time to 1min and
 * maintaining 25% duty cycle by having rest time = 4 * work time.
 * 2.) Stall procetion by checking if the previous position feedback value
 * is the same as the next. If the feedback value hasn't changed in 26ms,
 * the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * I use STALL_TIME instead of 26ms just to be sure that the bit value
 * has changed.
 */
void raiseTheActuator (byte currentHeight, byte goalHeight) {
  Serial.println("----Raise----");
  
  // note the time we start raising the actuator
  unsigned long startTime = millis();
  int stallCheckPosition = analogRead(FEEDBACK);

  // begin raising the actuator
  digitalWrite(MOTOR_F, HIGH);
  digitalWrite(MOTOR_B, LOW);

  while (currentHeight < goalHeight && millis() < (startTime + MAX_WORK_TIME)) {

    Serial.print("analogRead = ");
    Serial.println(analogRead(FEEDBACK));
    Serial.print("currentHeight = ");
    Serial.println(currentHeight);
    Serial.println();
    
    // stall check 
    // TODO commented out for testing. Remove comments when done.
//        delay(STALL_TIME);
//        if (stallCheckPosition == analogRead(FEEDBACK)) {
//          Serial.print("motor has stalled at: ");
//          Serial.println(stallCheckPosition);
//          stopTheActuator(0);
//          break;
//        }
//        stallCheckPosition = analogRead(FEEDBACK);

    // TODO what happens when currentHeight overshoots goalHeight
    // and is higher instead of equal?
    currentHeight = getCurrentHeight();

    // TODO this is a hack because ROS has a sample rate of 10Hz, the arduino is working at 115200baud which is 115kHz
    // so the ROS server can not keep up with all the messages the arduino is publishing
    // delete this when you have time to think of a better  solution.
    delay(150);

    sendFeedback(currentHeight);
  }

  // Overheat protection
  unsigned long workTime = millis() - startTime;
  Serial.print("Worked for: ");
  Serial.print(workTime/1000);
  Serial.println("s");
  unsigned long restTime = 4 * workTime;

  stopTheActuator(restTime);
}


/**
 * There are two safety mechanisms implemented in the lower function.
 * 1.) Overheat protection by limiting the maximum work time to 1min and
 * maintaining 25% duty cycle by having rest time = 4 * work time.
 * 2.) Stall procetion by checking if the previous position feedback value
 * is the same as the next. If the feedback value hasn't changed in 26ms,
 * the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * I use delay(50) instead of 26ms just to be sure that the bit value
 * has changed.
 */
void lowerTheActuator (byte currentHeight, byte goalHeight) {
  Serial.println("----Lower----");
  
  // note the time we start lowering the actuator
  unsigned long startTime = millis();
  int stallCheckPosition = analogRead(FEEDBACK);
  
  // begin lowering the actuator
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, HIGH);

  while (currentHeight > goalHeight && millis() < (startTime + MAX_WORK_TIME)) {

    Serial.print("analogRead = ");
    Serial.println(analogRead(FEEDBACK));
    Serial.print("currentHeight = ");
    Serial.println(currentHeight);
    Serial.println();
    
    // stall protection
    // TODO commented out for testing
//        delay(STALL_TIME);
//        if (stallCheckPosition == analogRead(FEEDBACK)) {
//          Serial.print("motor has stalled at: ");
//          Serial.println(stallCheckPosition);
//          stopTheActuator(0);
//          break;
//        }
//        stallCheckPosition = analogRead(FEEDBACK);

    // TODO what happens when currentHeight overshoots goalHeight
    // and is lower instead of equal?
    currentHeight = getCurrentHeight();

    // TODO this is a hack because ROS has a sample rate of 10Hz, the arduino is working at 115200baud which is 115kHz
    // so the ROS server can not keep up with all the messages the arduino is publishing
    // delete this when you have time to think of a better  solution.
    delay(150);

    sendFeedback(currentHeight);
  }

  // Overheat protection
  unsigned long workTime = millis() - startTime;
  Serial.print("Worked for: ");
  Serial.print(workTime/1000);
  Serial.println("s");
  unsigned long restTime = 4 * workTime;
  
  stopTheActuator(restTime);
}


/**
 * Stop the actuator and print the time it will rest.
 */
void stopTheActuator(unsigned long restTime) {

  // stop the actuator
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, LOW);

  // send feedback of the final seat height
  // TODO commented out for testing purposes
  // What happens if the actuator overshoots the desired position before it stops?
  
  //sendFeedback(getCurrentHeight());
  
  // Print the resting time to the Serial output
  Serial.print("Resting for: ");
  Serial.print(restTime/1000);
  Serial.println("s");
  delay(restTime);  // replace with function which checks for incoming messages and
                    // returns the current height with status 'C' for cooldown. 
                    // Next version of this function should return the time left to rest
}

/**
 * Send feedback to the ROS on board computer over CAN.
 * The feedback message contains one byte type number 
 * from 0 to 150 indicating the current height of the 
 * seat in millimeters.
 */
void sendFeedback(byte currentHeight) {
  // TODO expand the data length to include a status byte after the first byte which is the height
  unsigned char feedbackMsg[1] = {currentHeight};

  // uC send data:  0x111 = id, 0 = standard frame, 1 = data length, feedbackMsg = data buf
  CAN.sendMsgBuf(0x111, 0, 1, feedbackMsg);
}

/**
 * READ FROM SERIAL INPUT
 * DEBUG FUNCTION
 */
void readFromSerial() {
   
  if (Serial.available() > 0) {
        byte goalHeight = Serial.parseInt();
        Serial.read(); // remove new line /n from serial
        byte currentHeight = getCurrentHeight();
        
        SERIAL.println("-----------------------------");
        SERIAL.print("Goal Height: ");
        SERIAL.println(goalHeight);
        SERIAL.print("Current Height: ");
        SERIAL.println(currentHeight);
        SERIAL.println();

        if (currentHeight < goalHeight) raiseTheActuator(currentHeight, goalHeight);
        if (currentHeight > goalHeight) lowerTheActuator(currentHeight, goalHeight);
        if (currentHeight == goalHeight) stopTheActuator(0);

        // ToDo send current height to CAN BUS

        Serial.println("Please enter new goalHeight");
    }
}

/**
 * Read message from CAN bus.
 * Message data is sent from ROS as a string. 
 * The readMsgBuf function transforms the characters from hex to decimal
 * atoi() converts those character numbers into a single int number
 * Example, sending goal height 100mm from ROS to Arduino:
 * "100" -> 31 30 30 -> 49 48 48 -> 100
 * 
 * Messages sent from ROS have canId = 0x555
 * Messages sent from Arduino have canId = 0x111
 * The arduino should not read messages which have not been sent by ROS.
 */
void readFromCanBus() {
  unsigned char len = 0; //msg data length
  unsigned char buf[8];  //msg data
    
  if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data is comming
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    // read messages published only by ROS
    if (canId == (int)ROS_CAN_ID) {
    
      byte goalHeight = int(buf[0]); 
      byte currentHeight = getCurrentHeight();
  
      SERIAL.println("-----------------------------");
      SERIAL.print("Get data from ID: 0x");
      SERIAL.println(canId, HEX);
      SERIAL.print("buf = ");
      for (int i=0;i<len;i++){
        SERIAL.print(buf[i]);
        SERIAL.print(" ");
      }
      SERIAL.println();
      SERIAL.print("len = ");
      SERIAL.println(len);
      SERIAL.print("Goal Height: ");
      SERIAL.println(goalHeight);
      SERIAL.print("Current Height: ");
      SERIAL.println(currentHeight);
  
      // ToDo test all cases
      if (currentHeight == goalHeight) stopTheActuator(0);
      else if (currentHeight < goalHeight) raiseTheActuator(currentHeight, goalHeight);
      else if (currentHeight > goalHeight) lowerTheActuator(currentHeight, goalHeight);
    }
    else {
      Serial.print("Ignoring message with id = ");
      Serial.println(canId);
    }
    
  }
}