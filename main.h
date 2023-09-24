// pinout https://blog.protoneer.co.nz/arduino-cnc-shield/
// current control http://www.360doc.com/content/20/0805/03/12109864_928572399.shtml
// float to hex https://gregstoll.com/~gregstoll/floattohex/



#include <Arduino.h>
#include <EEPROM.h> // Note: An EEPROM write takes 3.3 ms to complete. The EEPROM memory has a specified life of 100,000 write/erase cycles, so you may need to be careful about how often you write to it.
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "kinematics.cpp"

#define debug
#define DirJoint0 -1.0f
#define DirJoint1 -1.0f
#define DirJoint2 -1.0f
#define DirJoint3 1.0f



//SERIAL_BUFFER_SIZE = 64
const int INSTRUCTION_SIZE = 1; // Instruction
uint8_t INSTRUCTION_BUFFER[INSTRUCTION_SIZE]; // Instruction buffer
const int FLOAT_SIZE = 4; // float 4 bytes
uint8_t FLOAT_BUFFER[FLOAT_SIZE];

// static 
// constant
bool PinStatus = false, ExButtonStatus = true; // True: logic low, False: logic high



struct position {
  float joint0, joint1, joint2, joint3;
  float X, Y, Z;
  float A, B, C;
};

#ifdef debug
  uint8_t debug_INSTRUCTION_BUFFER;
  bool debug_PinStatus;
  bool debug_pin9;
  bool debug_pin10;
  bool debug_pin11;
  bool debug_pinA1;
#endif

// BasicStepperDriver(short steps, short dir_pin, short step_pin);
BasicStepperDriver Joint0(200*27, 5, 2);
BasicStepperDriver Joint1(200*51, 6, 3);
BasicStepperDriver Joint2(200*27, 7, 4);
BasicStepperDriver Joint3(200, 12, 13);
SyncDriver controller(Joint0, Joint1, Joint2);

position Position;
position Target;
position Difference;

kinematics Kinematics(274,130);
