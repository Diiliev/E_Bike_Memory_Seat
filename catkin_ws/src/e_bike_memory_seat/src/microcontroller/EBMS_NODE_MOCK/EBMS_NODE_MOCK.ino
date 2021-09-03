// CAN recieve tutorial by loovee, 2014-6-13


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


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup() {
    SERIAL.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");

    // initialise motor output pins and set to low.
    pinMode(MOTOR_F, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    digitalWrite(MOTOR_F, LOW);
    digitalWrite(MOTOR_B, LOW);
}


void loop() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data is comming
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned long canId = CAN.getCanId();
        int goalHeight = atoi(buf);
        char driveDirection;
        
        SERIAL.println("-----------------------------");
        SERIAL.print("Get data from ID: 0x");
        SERIAL.println(canId, HEX);
        SERIAL.print("Goal Height: ");
        SERIAL.println(goalHeight);
        
        for (int i = 0; i < len; i++) { // print the data
          if(!isDigit(buf[i])) {
            driveDirection = buf[i];
            break;
          }
        }

        SERIAL.print("Direction: ");
        SERIAL.println(driveDirection);
        SERIAL.println();

        if (driveDirection == 'r') raiseTheActuator();
        if (driveDirection == 'l') lowerTheActuator();
        if (driveDirection == 's') stopTheActuator();
    }
}

void raiseTheActuator () {
  digitalWrite(MOTOR_F, HIGH);
  digitalWrite(MOTOR_B, LOW);
}

void lowerTheActuator () {
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, HIGH);
}

void stopTheActuator() {
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, LOW);
}
