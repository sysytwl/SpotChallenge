// pinout https://blog.protoneer.co.nz/arduino-cnc-shield/
// current control http://www.360doc.com/content/20/0805/03/12109864_928572399.shtml
// float to hex https://gregstoll.com/~gregstoll/floattohex/

#include "main.h"
#include "kinematics.cpp"

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

ISR(TIMER1_COMPA_vect){
  // get Instruction from the serial port
  if (Serial.available() > 0 && INSTRUCTION_BUFFER[0] == 0x03) { // todo: condiser about 0x00 status
    Serial.readBytes(INSTRUCTION_BUFFER, INSTRUCTION_SIZE);
    if (INSTRUCTION_BUFFER[0] == 0xFF){
      auto StepsRemain = controller.stop();
      float StepsRemainJoint3 = Joint3.stop();

      Position.joint0 = Position.joint0 - 0.0667f * (float) StepsRemain.steps[0];
      Position.joint1 = Position.joint1 - 0.0667f * (float) StepsRemain.steps[1];
      Position.joint2 = Position.joint2 - 0.0667f * (float) StepsRemain.steps[2];
      Position.joint3 = Position.joint3 - 0.0667f * StepsRemainJoint3;
    } else if (INSTRUCTION_BUFFER[0] == 0xFE) {
      PinStatus = true;
    }else {// todo: put the unknow part back to the pool
      INSTRUCTION_BUFFER[0] = 0x03;
    }
  }

  // Check the physical stop signal
  bool ButtonStatus = digitalRead(A0);
  if (ExButtonStatus == true && ButtonStatus == false) {
    PinStatus = !PinStatus;
    digitalWrite(8, PinStatus);
  }
  ExButtonStatus = ButtonStatus;

  TCNT1  = 0; // set the timer back to 0 so it resets for next interrupt // todo: change the time cost depend on time cost
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(90); // todo: check
  Serial.println("Type 0x05 for help.");

  // void BasicStepperDriver::begin(float rpm = (60.0F), short microsteps = (short)1)
  Joint0.begin(2, 1);
  Joint1.begin(1, 1);
  Joint2.begin(2, 1);
  Joint3.begin(90, 1);

  // Limit switch
  pinMode(9, INPUT_PULLUP); // Joint0
  pinMode(10, INPUT_PULLUP); // Joint1
  pinMode(11, INPUT_PULLUP); // Joint2
  pinMode(A1, INPUT_PULLUP); // Joint3

  // EmergencyStop
  pinMode(A0, INPUT_PULLUP);

  // Stepper Enable/Disable
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  // Time IRS
  cli();                      //stop interrupts for till we make the settings
  //TCCR1A = 0;                 // Reset entire TCCR1A to 0
  TCCR1B = 0; 
  TCCR1B |= B00000101;        //Set CS12 to 1 so we get prescalar 1024  
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  OCR1A = 1563; // 0.100032s
  sei();                      //Enable back the interrupts

  // set joints to -999
  Position.joint0 = -999;
  Position.joint1 = -999;
  Position.joint2 = -999;
  Position.joint3 = -999;

  // set Status to Init
  INSTRUCTION_BUFFER[0] = 0x00;
  #ifdef debug
    INSTRUCTION_BUFFER[0] = 0x01;
  #endif
}

void loop(){
  switch (INSTRUCTION_BUFFER[0]) {
  case 0x00: // Init
    if (!digitalRead(9)) {Position.joint0 = 180;}
    if (!digitalRead(10)) {Position.joint1 = 0;}
    if (!digitalRead(11)) {Position.joint2 = 0;}
    //if (!digitalRead(A1)){Position.joint3 = 0;} todo!!
    Position.joint3 = 0;

    // exp0 ? T : F
    controller.rotate(
      (Position.joint0 == 180) ? 0 : DirJoint0 * 1,
      (Position.joint1 == 0) ? 0 : DirJoint1 * -1,
      (Position.joint2 == 0) ? 0 : DirJoint2 * -1
    );

    if (Position.joint0 == 180 && Position.joint1 == 0 && Position.joint2 == 0 && Position.joint3 == 0){
      Serial.println("Ready. 0x05 for help.");
      Difference.joint0 = -180;
      Difference.joint1 = Difference.joint2 = Difference.joint3 = 0;
      INSTRUCTION_BUFFER[0] = 0x03;
    }
    break;

  case 0x01: // Idle
    if (Serial.available()) {Serial.readBytes(INSTRUCTION_BUFFER, INSTRUCTION_SIZE);}
    break;

  case 0x02: // Goto
    if (Serial.available() < 16) {
      delay(100); // wait
      Serial.println("A float contains 4 bytes * 4 Joints!");
      break;
    }

    /* This method is not working
    *((uint8_t*) (&Target.joint1    )) = JOINT_BUFFER[0];
    *((uint8_t*) (&Target.joint1 + 1)) = JOINT_BUFFER[1];
    *((uint8_t*) (&Target.joint1 + 2)) = JOINT_BUFFER[2];
    *((uint8_t*) (&Target.joint1 + 3)) = JOINT_BUFFER[3];
    */

    // Joint0
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.joint0, FLOAT_BUFFER, 4);

    // Joint1
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.joint1, FLOAT_BUFFER, 4);

    // Joint2
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.joint2, FLOAT_BUFFER, 4);

    // Joint3
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.joint3, FLOAT_BUFFER, 4);

    Difference.joint0 = Target.joint0 - Position.joint0;
    Difference.joint1 = Target.joint1 - Position.joint1;
    Difference.joint2 = Target.joint2 - Position.joint2;
    Difference.joint3 = Target.joint3 - Position.joint3;

    INSTRUCTION_BUFFER[0] = 0x03;
    break;

  case 0x03: // Working
    if (Position.joint0 < -180 || Position.joint1 < 0 || Position.joint2 < 0 || Position.joint3 < 0) {
      Serial.println("Init required");
    }else if (Target.joint0 > 180 || Target.joint0 < -180 || Target.joint1 < 0 || Target.joint1 > 180 || Target.joint2 < 0 || Target.joint2 > 180 || Target.joint3 < -180 || Target.joint3 > 180) {
      Serial.println("Out of range!");
    } else {
      #ifdef debug
        Serial.print("Goto: ");
        Serial.print(Target.joint0); Serial.print(" ("); Serial.print(Difference.joint0); Serial.print(") ");
        Serial.print(Target.joint1); Serial.print(" ("); Serial.print(Difference.joint1); Serial.print(") ");
        Serial.print(Target.joint2); Serial.print(" ("); Serial.print(Difference.joint2); Serial.print(") ");
        Serial.print(Target.joint3); Serial.print(" ("); Serial.print(Difference.joint3); Serial.print(") ");
        Serial.println();
      #endif

      controller.rotate(DirJoint0 * Difference.joint0,DirJoint1 * Difference.joint1,DirJoint2 * Difference.joint2);
      Joint3.rotate(DirJoint3 * Difference.joint3);

      Position.joint0 += Difference.joint0;
      Position.joint1 += Difference.joint1;
      Position.joint2 += Difference.joint2;
      Position.joint3 += Difference.joint3;

      Serial.println("Finished.");
    }

    INSTRUCTION_BUFFER[0] = 0x01;
  break;

  case 0x04: // Inverse kinematics
    if (Serial.available() < 12) { // 1-4: X, 5-8: Y, 9-12: Z, 13-16: DirectionX, 17-20: DirectionY, 21-24: DirectionZ
      delay(100); // wait
      Serial.println("A float contains 4 bytes * 3 Axis!");
      break;
    }

    // X
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.X, FLOAT_BUFFER, 4);

    // Y
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.Y, FLOAT_BUFFER, 4);

    // Z
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.Z, FLOAT_BUFFER, 4);

    /* todo??? rotation vector
    // DirectionX, A
    // DirectionY, B
    // DirectionZ, C09
    */

    // calculate
    Kinematics.InverseKinematics(Target.X, Target.Y, Target.Z);
    
    if (Kinematics.Position_.Lefty) { // todo: consider about the angle at the end point
      Target.joint0 = Kinematics.Position_.LeftyJoint0;
      Target.joint1 = Kinematics.Position_.LeftyJoint1;
      Target.joint2 = Kinematics.Position_.LeftyJoint2;
      Target.joint3 = Kinematics.Position_.LeftyJoint3;
    } else if (Kinematics.Position_.Rightly) {
      Target.joint0 = Kinematics.Position_.RightlyJoint0;
      Target.joint1 = Kinematics.Position_.RightlyJoint1;
      Target.joint2 = Kinematics.Position_.RightlyJoint2;
      Target.joint3 = Kinematics.Position_.RightlyJoint3;
    } else {
      Serial.println("Unreachable");
      INSTRUCTION_BUFFER[0] = 0x01;
      break;
    }

    Difference.joint0 = Target.joint0 - Position.joint0;
    Difference.joint1 = Target.joint1 - Position.joint1;
    Difference.joint2 = Target.joint2 - Position.joint2;
    Difference.joint3 = Target.joint3 - Position.joint3;

    INSTRUCTION_BUFFER[0] = 0x03;
  break;

  case 0x05:  // Help
    Serial.print("Init: 0x00\nIdle: 0x01\nGoto: 0x02\nWorking: 0x03\nInverse kinematics: 0x04\nHelp: 0x05\nForward kinematics: 0x06\nCurrent joint angle: 0x07\nTurn certain angle: 0x08\nmanual init: 0x09\nInverse kinematics based on top end: 0x10\nEmergency stop: 0xFE\nStop: 0xFF");
    INSTRUCTION_BUFFER[0] = 0x01;
    break;

  case 0x06: // Forward kinematics
    Kinematics.ForwardKinematics(Position.joint0, Position.joint1, Position.joint2);
    Serial.print("Current endpoint position: ");
    Serial.print("X: ");
    Serial.print(Kinematics.ForwardKinematics_.X);
    Serial.print("  Y: ");
    Serial.print(Kinematics.ForwardKinematics_.Y);
    Serial.print("  Z: ");
    Serial.print(Kinematics.ForwardKinematics_.Z);
    INSTRUCTION_BUFFER[0] = 0x01;
    break;

  case 0x07: // Current joint angle
    Serial.print("Current joint angle: ");
    Serial.print(Position.joint0); Serial.print(", ");
    Serial.print(Position.joint1); Serial.print(", ");
    Serial.print(Position.joint2); Serial.print(", ");
    Serial.println(Position.joint3);

    INSTRUCTION_BUFFER[0] = 0x01;
    break;

  case 0x08: // Turn certain angle
    if (Serial.available() < 16) {
      delay(100); // wait
      Serial.println("A float contains 4 bytes * 4 Joints!");
      break;
    }

    // Joint0
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Difference.joint0, FLOAT_BUFFER, 4);

    // Joint1
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Difference.joint1, FLOAT_BUFFER, 4);

    // Joint2
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Difference.joint2, FLOAT_BUFFER, 4);

    // Joint3
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Difference.joint3, FLOAT_BUFFER, 4);

    Target.joint0 = Position.joint0 + Difference.joint0;
    Target.joint1 = Position.joint1 + Difference.joint1;
    Target.joint2 = Position.joint2 + Difference.joint2;
    Target.joint3 = Position.joint3 + Difference.joint3;

    INSTRUCTION_BUFFER[0] = 0x03;
    break;

  case 0x09: // manual init
    Position.joint0 = 0;
    Position.joint1 = 0;
    Position.joint2 = 0;
    Position.joint3 = 0;
    PinStatus = true;
    INSTRUCTION_BUFFER[0] = 0x01;
  break;

  case 0x10: // Inverse kinematics based on top end
    if (Serial.available() < 12) { // 1-4: X, 5-8: Y, 9-12: Z, 13-16: DirectionX, 17-20: DirectionY, 21-24: DirectionZ
    delay(100); // wait
    Serial.println("A float contains 4 bytes * 3 Axis!");
    break;
  }

    // X
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.X, FLOAT_BUFFER, 4);

    // Y
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.Y, FLOAT_BUFFER, 4);

    // Z
    Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
    memcpy (&Target.Z, FLOAT_BUFFER, 4);

    Kinematics.ForwardKinematics(Position.joint0, Position.joint1, Position.joint2);
    Kinematics.CoordinateTrans(-Kinematics.ForwardKinematics_.X, -Kinematics.ForwardKinematics_.Y, -Kinematics.ForwardKinematics_.Z, Target.X, Target.Y, Target.Z);
    Target.X = Kinematics.transformed_.X;
    Target.Y = Kinematics.transformed_.Y;
    Target.Z = Kinematics.transformed_.Z;
    Kinematics.InverseKinematics(Target.X, Target.Y, Target.Z);
    
    if (Kinematics.Position_.Lefty) { // todo: consider about the angle at the end point
      Target.joint0 = Kinematics.Position_.LeftyJoint0;
      Target.joint1 = Kinematics.Position_.LeftyJoint1;
      Target.joint2 = Kinematics.Position_.LeftyJoint2;
      Target.joint3 = Kinematics.Position_.LeftyJoint3;
    } else if (Kinematics.Position_.Rightly) {
      Target.joint0 = Kinematics.Position_.RightlyJoint0;
      Target.joint1 = Kinematics.Position_.RightlyJoint1;
      Target.joint2 = Kinematics.Position_.RightlyJoint2;
      Target.joint3 = Kinematics.Position_.RightlyJoint3;
    } else {
      Serial.println("Unreachable");
      INSTRUCTION_BUFFER[0] = 0x01;
      break;
    }

    Difference.joint0 = Target.joint0 - Position.joint0;
    Difference.joint1 = Target.joint1 - Position.joint1;
    Difference.joint2 = Target.joint2 - Position.joint2;
    Difference.joint3 = Target.joint3 - Position.joint3;

    INSTRUCTION_BUFFER[0] = 0x03;

    break;

    case 0xFE:
      Serial.println("Estop.");
      PinStatus = true;
      INSTRUCTION_BUFFER[0] = 0x01;
    break;

    case 0xFF:
      Serial.println("Stop.");
      INSTRUCTION_BUFFER[0] = 0x01;
    break;

  default:
    INSTRUCTION_BUFFER[0] = 0x01;
  break;
  }
}
