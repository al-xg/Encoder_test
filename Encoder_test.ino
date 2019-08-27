//Motor + encoder////////////////////////////////////////////////////////////////////////////////////

//https://www.pjrc.com/teensy/td_libs_Encoder.html Teensy encoder library
//https://www.pololu.com/product/3081 Pololu encoder
//https://www.pololu.com/product/2130 Dual Motor Driver Carrier
//https://www.pololu.com/file/0J534/drv8833.pdf DRV8833 datasheet

#include <PID_v1.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Ramp.h>

int MotorFwd = 10;                 // Connections to motor driver board
int MotorBck = 9;                  // Connections to motor driver board
//bool dir = 0;                    // Rotation direction state 0=CW 1=CCW
bool StopStart = 0;                // Enables or disables motor outputs
int GBRatio = 100;                 // Gearmotor ratio 100:1
long AngleCommand = 0;             // Output shaft angle to aim for in position control
double PositionCommand = 0;        // Number of encoder counts to aim for in position control
double PrevPosition = 0;           // Holds the current position command as the starting point for ramping
unsigned long MotionDuration = 15; // Target time (ms) in which the controller shouwld try and reach PositionCommand
long RampedPosition = 0;           // Intermediate target positions determined by ramp library
bool updateRamp = true;            // Only update Ramp function when new position or duration is received
bool Decay = HIGH;                 // LOW= fast decay, HIGH= slow decay -- controls motor breaking

Encoder M1(1, 0);
long positionM1  = -999;
int CPR = 12;                      // Number of encoder counts per motor revolution (12 for Polulu encoder)
long revCount = 0;                 // For counting revolutions
long M1Count = 0;                  // Motor 1 encoder counts
long prevCount = 0;
const long EncToAngle = 2777 * GBRatio * CPR; // Mulitplied by 1000000 to remove decimals

rampDouble M1Ramp;                 // new ramp object (ramp<unsigned char> by default)

//PID////////////////////////////////////////////////////////////////////////////////////////////

//https://playground.arduino.cc/Code/PIDLibrary/

double Setpoint, Input, Output, Kp = 7, Ki = 150, Kd = 0.15;
PID M1PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Setup///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Coms
  Serial.begin(115200);

  // Motor
  pinMode(MotorFwd, OUTPUT); // Declare pin 10 to be an output
  pinMode(MotorBck, OUTPUT); // Declare pin 9 to be an output

  // Encoder
  M1.write(M1Count);

  // PID
  Input = M1Count;
  Setpoint = M1Count;
  M1PID.SetMode(AUTOMATIC);
  M1PID.SetSampleTime(10);
  M1PID.SetOutputLimits(-255, 255);
}

//Loop//////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  GetSerialCommands();
  //ReadEncoder();

  if (updateRamp == true) {
    M1Ramp.go(PositionCommand, MotionDuration, LINEAR, ONCEFORWARD);
    updateRamp = false;
  }
  RampedPosition = M1Ramp.update();

  Setpoint = RampedPosition;
  M1Count = M1.read();
  Input = M1Count;
  M1PID.Compute();

  OutputPWM();
  plotThings();
}

//Functions///////////////////////////////////////////////////////////////////////////////////////////
void OutputPWM() {
  if (StopStart == 1) {
    M1Ramp.resume();
    int power = abs(Output);    // How much power is being applied to motor. Movement range +/- ~20-255
    if (Output < 0) {
      //Reverse PWM
      analogWrite(MotorFwd, Decay);
      analogWrite(MotorBck, power);
    } else {
      //Forward PWM
      analogWrite(MotorBck, Decay);
      analogWrite(MotorFwd, power);
    }
  } else {
    M1Ramp.pause();
    Setpoint = M1Count;         // Stop the PID output from building up until the motors are active
    analogWrite(MotorFwd, Decay);
    analogWrite(MotorBck, Decay);
  }
}

// Use Ramp library to interpolate through to PositionCommand over the time of MotionDuration
int Ramp(double PositionCommand, long MotionDuration) {
  M1Ramp.go(PositionCommand, MotionDuration, LINEAR, ONCEFORWARD);
  return M1Ramp.update();
}


void ReadEncoder() {            // Function to report the encoder position and motor rotations
  long newM1;
  newM1 = M1.read();
  /*if (newM1 >= (CPR + 1)) {
    newM1 = 0;
    M1.write(0);
    revCount = revCount + 1;
    Serial.print("Rev = ");
    Serial.print(revCount);
    Serial.println();
    }

    if (newM1 <= (-CPR - 1)) {
    newM1 = 0;
    M1.write(0);
    revCount = revCount - 1;
    Serial.print("Rev = ");
    Serial.print(revCount);
    Serial.println();
    }
  */
  if (newM1 != positionM1) {
    Serial.print("M1 = ");
    Serial.print(newM1);
    Serial.println();
    positionM1 = newM1;
  }
}
