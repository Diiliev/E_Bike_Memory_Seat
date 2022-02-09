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
#define FEEDBACK 0
#define MAX_WORK_TIME 60000
#define STALL_TIME 1000 // In theory this should be 26ms but in reality the time it takes to move the actuator with 1mm is ~135ms.
#define ROS_CAN_ID 0x555
#define STALLED_CODE 255
#define ACTUATOR_STOP_TIME 100


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
   *  Return the current height in mm instead of bytes
   *  1023b / 150 = 6.82
   *  1b = 0.147mm
   *  1mm = 6.82b
   *  when casting to byte the value is rounded
   *  and halves are zeroed which means   
   *  100.1 is rounded to 100
   *  100.5 is rounded to 101 
   *  100.9 is rounded to 101
   */
byte getCurrentHeight() {
  
  float analogReading = analogRead(FEEDBACK);
  byte roundedCurrentHeight = round(analogReading / 6.82);
  return roundedCurrentHeight;
}

/**
 * There are four safety mechanisms implemented in the raising function.
 * 1.) Overheat protection by limiting the maximum work time to 1min 
 * 2.) Overheat protection by maintaining 25% duty cycle by having rest time = 4 * work time.
 * 3.) Stall procetion by checking if the previous position feedback value
 *     is the same as the next. If the feedback value hasn't changed in 26ms,
 *     the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * 4.) Overshoot protection.
 *     If the current height overshots the goal and is now higher than the goal height, 
 *     lower the actuator and also continue tracking the work time of the actuator. 
 *     Otherwise if we have reached the goal height, send the final feedback message and let the actuator rest.
 *     Otherwise, if the actuator has stalled in a position where currentHeight > goalHeight
 *     we have already sent the feedback message that it has stalled.
 *     Now we only need to let the actuator rest without sending any more messages.
 * 
 * If this function is called after previously lowering the actuator,
 * then it has already worked for an "elapsedWorkTime" ammount of time.
 * This time needs to be accounted for in the time it takes to now raise the actuator
 * and the time it takes to rest after the operation has finished.
 * 
 * This function sends feedback messages only if the current height has changed and
 *  it is higher than the previously recorded current height. This is done for several reasons:
 *  - To prevent spamming messages with the same value.
 *  - To prevent us from sending flip-flop values, i.e when the actuator is in a position between
 *    two values and the reading changes from one to the other.
 *  - To prevent us from sending a feedback message where the current height
 *    is equal to the goal height before we have stopped the actuator in that position.
 *    Otherwise we risk notifying ROS that we have reached the goal height, overshoot it and
 *    send new feedback messages, while ROS is already accepting new goals.
 * 
 * TODO add a check if the while loop has ended because it reached the goal height
 *  or because the availableWorkTime was not sufficient to finish the operation.
 *  
 * TODO implement feedback messages with more detailed status such as:
 *  - stalled, timeout, resting, etc.
 *  
 */
void raiseTheActuator (byte currentHeight, byte goalHeight, unsigned long elapsedWorkTime) {
  
  Serial.println("----Raise----");

  byte currentHeightPreviousValue = currentHeight; // this is in millimeters from 0 to 150
  unsigned long availableWorkTime = MAX_WORK_TIME - elapsedWorkTime; // Calculate the available work time the actuator has left.
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;

  // begin raising the actuator
  digitalWrite(MOTOR_F, HIGH);
  digitalWrite(MOTOR_B, LOW);
  
  while (currentHeight < goalHeight && millis() < (startTime + availableWorkTime)) {

    // stall protection 
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      Serial.print("Actuator has stalled at: ");
      Serial.print(getCurrentHeight());
      Serial.println("mm");
      break;
    }

    currentHeight = getCurrentHeight();

    // send feedback only if the current height has changed and it is not the goal height.
    // After that reset currentHeightPreviousValue
    if (currentHeight > currentHeightPreviousValue && currentHeight < goalHeight) {
      sendFeedback(currentHeight);
      Serial.print("currentHeight = ");
      Serial.println(currentHeight);
      currentHeightPreviousValue = currentHeight;
      stallTimer = millis() + STALL_TIME;
    }

    // without this delay the function doesn't work.
    // I suspect the loop spins too fast for for the microcontroller to handle.
    delay(10);
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime + elapsedWorkTime;
  Serial.print("Worked for: ");
  Serial.print(workTime/1000);
  Serial.println("s");

  // Check if position is overshot, successful on goal or failed due to stall.
  if (currentHeight > goalHeight) {
    lowerTheActuator(currentHeight, goalHeight, workTime);
  }
  else if(currentHeight == goalHeight){
    sendFeedback(currentHeight);
    Serial.print("Success! Final position is ");
    Serial.print(currentHeight);
    Serial.println("mm");
    restTheActuator(workTime);
  }
  else {
    Serial.print("Failed! Final position is ");
    Serial.print(currentHeight);
    Serial.println("mm");
    restTheActuator(workTime);
  }
}


/**
 * There are four safety mechanisms implemented in the lower function.
 * 1.) Overheat protection by limiting the maximum work time to 1min 
 * 2.) Overheat protection by maintaining 25% duty cycle by having rest time = 4 * work time.
 * 3.) Stall procetion by checking if the previous position feedback value
 *     is the same as the next. If the feedback value hasn't changed in 26ms,
 *     the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * 4.) Overshoot protection.
 *     If the current height overshots the goal and is now lower than the goal height, 
 *     raise the actuator and also continue tracking the work time of the actuator. 
 *     Otherwise if we have reached the goal height, send the final feedback message and let the actuator rest.
 *     Otherwise, if the actuator has stalled in a position where currentHeight > goalHeight
 *     we have already sent the feedback message that it has stalled.
 *     Now we only need to let the actuator rest without sending any more messages.
 * 
 * If this function is called after previously raising the actuator,
 * then it has already worked for an "elapsedWorkTime" ammount of time.
 * This time needs to be accounted for in the time it takes to now lower the actuator
 * and the time it takes to rest after the operation has finished.
 * 
 * This function sends feedback messages only if the current height has changed and
 *  it is lower than the previously recorded current height. This is done for several reasons:
 *  - To prevent spamming messages with the same value.
 *  - To prevent us from sending flip-flop values, i.e when the actuator is in a position between
 *    two values and the reading changes from one to the other.
 *  - To prevent us from sending a feedback message where the current height
 *    is equal to the goal height before we have stopped the actuator in that position.
 *    Otherwise we risk notifying ROS that we have reached the goal height, overshoot it and
 *    send new feedback messages, while ROS is already accepting new goals.
 * 
 * TODO add a check if the while loop has ended because it reached the goal height
 *  or because the availableWorkTime was not sufficient to finish the operation.
 *  
 * TODO implement feedback messages with more detailed status such as:
 *  - stalled, timeout, resting, etc.
 */
void lowerTheActuator (byte currentHeight, byte goalHeight, unsigned long elapsedWorkTime) {
  
  Serial.println("----Lower----");
  
  byte currentHeightPreviousValue = currentHeight; // this is in millimeters from 0 to 150
  unsigned long availableWorkTime = MAX_WORK_TIME - elapsedWorkTime; // Calculate the available work time the actuator has left.
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;
  
  // begin lowering the actuator
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, HIGH);

  while (currentHeight > goalHeight && millis() < (startTime + availableWorkTime)) {

    // stall protection
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      Serial.print("Actuator has stalled at: ");
      Serial.print(getCurrentHeight());
      Serial.println("mm");
      break;
    }

    currentHeight = getCurrentHeight();
    
    // send feedback only if the current height has changed and it is not the goal height.
    // After that reset currentHeightPreviousValue and stallTimer
    if (currentHeight < currentHeightPreviousValue &&  currentHeight > goalHeight) {
      sendFeedback(currentHeight);
      Serial.print("currentHeight = ");
      Serial.println(currentHeight);
      currentHeightPreviousValue = currentHeight;
      stallTimer = millis() + STALL_TIME;
    }

    // without this delay the function doesn't work.
    // I suspect the loop spins too fast for for the microcontroller to handle.
    // TODO figure out why the fuction does not work without this delay.
    delay(10); 
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime + elapsedWorkTime;
  Serial.print("Worked for: ");
  Serial.print(workTime/1000);
  Serial.println("s");

  // Check if position is overshot, successful on goal or failed due to stall.
  if (currentHeight < goalHeight) {
    raiseTheActuator(currentHeight, goalHeight, workTime);
  }
  else if(currentHeight == goalHeight){
    sendFeedback(currentHeight);
    Serial.print("Success! Final position is ");
    Serial.print(currentHeight);
    Serial.println("mm");
    restTheActuator(workTime);
  }
  else {
    Serial.print("Failed! Final position is ");
    Serial.print(currentHeight);
    Serial.println("mm");
    restTheActuator(workTime);
  }
}


/**
 * Stop the actuator immediatelly.
 */
void stopTheActuator() {
  
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, LOW);
}

/**
 * Calculate the appropriate ammount of rest time from the given work time.
 * Given that the duty cycle of the actuator is 25%, that means it must rest for
 * 4 times the ammount of time spent working.
 * Stop the execution of the program for "restTime" ammount of time
 * and print "restTime" to the Serial monitor for debugging purposes.
 * 
 * TODO replace this with a function which checks for incoming messages and
 *      returns the current height with status 'C' for cooldown. 
 *      Next version of this function should also return the time left to rest
 * 
 * TODO send feedback to ROS when resting is complete.
 */
void restTheActuator(unsigned long workTime) {
  
  unsigned long restTime = 4 * workTime;
  Serial.print("Resting for: ");
  Serial.print(restTime/1000);
  Serial.println("s");
  delay(restTime); 
  Serial.println("Resting complete.");
}

/**
 * Send feedback to the ROS on board computer over CAN.
 * The feedback message contains one byte type number 
 * from 0 to 150 indicating the current height of the 
 * seat in millimeters.
 * 
 * TODO expand the data length to include a status byte after the first byte which is the height
 */
void sendFeedback(byte currentHeight) {
  
  unsigned char feedbackMsg[1] = {currentHeight}; 
  CAN.sendMsgBuf(0x111, 0, 1, feedbackMsg); // uC send data:  0x111 = id, 0 = standard frame, 1 = data length, feedbackMsg = data buf
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

        if (currentHeight < goalHeight) raiseTheActuator(currentHeight, goalHeight, 0);
        if (currentHeight > goalHeight) lowerTheActuator(currentHeight, goalHeight, 0);
        if (currentHeight == goalHeight) {
          stopTheActuator();
          sendFeedback(currentHeight);
        }

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
  
      if (currentHeight == goalHeight) {
        stopTheActuator();
        sendFeedback(currentHeight);
      }
      else if (currentHeight < goalHeight) raiseTheActuator(currentHeight, goalHeight, 0);
      else if (currentHeight > goalHeight) lowerTheActuator(currentHeight, goalHeight, 0);
    }
    else {
      Serial.print("Ignoring message with id = ");
      Serial.println(canId);
    }
    
  }
}
