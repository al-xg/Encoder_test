//Motor + encoder////////////////////////////////////////////////////////////////////////////////////

//https://www.pjrc.com/teensy/td_libs_Encoder.html
//https://www.pololu.com/product/3081 Pololu encoder
//https://www.pololu.com/product/2130 Motor driver

#include <PID_v1.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

int MotorFwd = 10;         // Connections to motor driver board
int MotorBck = 9;          // Connections to motor driver board
bool dir = 0;              // Rotation direction state 0=CW 1=CCW
bool StopStart = 0;        //Enables or disables motor outputs
int GBRatio = 100;         // Gearmotor ratio 100:1
double power = 0;          // How much power is being applied to motor. Movement range +/- ~20-255
long AngleCommand = 0;     //Output shaft angle to aim for in position control
double PositionCommand = 0;  //Number of encoder counts to aim for in position control

Encoder M1(1, 0);
long positionM1  = -999;
int CPT = 12;                //Number of encoder counts per motor revolution (12 for Polulu encoder)
long revCount = 0;          //For counting revolutions
long M1Count = 0;           //Motor 1 encoder counts
long prevCount = 0;


const long EncToAngle=2777*GBRatio*CPT; //mulitplied by 1000000 to remove decimals

//PID////////////////////////////////////////////////////////////////////////////////////////////

//https://playground.arduino.cc/Code/PIDLibrary/

double SetpointNom, Setpoint, Input, Output, Kp = 0.08, Ki = 0.15, Kd = 0.02;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool plot=0; //Turns on or off the PID plotting

//Setup///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Motor
  pinMode(MotorFwd, OUTPUT); // declare pin 10 to be an output
  pinMode(MotorBck, OUTPUT); // declare pin 9 to be an output

  //Encoder
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset M1 to zero");
    M1.write(0);
  }

  //PID
  Input = M1Count;
  Setpoint = M1Count;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  //Coms
  Serial.begin(115200);

}

//Loop//////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  SerialCommands();
  //ReadEncoder();
  M1Count=M1.read();
  if (plot == 1) {
    plotThings();
  }

  Input = M1Count;
  myPID.Compute();
  power = abs(Output);

  if (StopStart == 1) {
    if (dir == 1) {
      Setpoint = (PositionCommand * -1);
    } else {
      Setpoint = PositionCommand;
    }
    if (Output < 0) {
      analogWrite(MotorFwd, 0);
      //digitalWrite(MotorFwd, HIGH);
      analogWrite(MotorBck, power);
    } else {
      analogWrite(MotorBck, 0);
      //digitalWrite(MotorBck, HIGH);
      analogWrite(MotorFwd, power);
    }
  } else {
    Setpoint = M1Count;
    power = 0;
    analogWrite(MotorFwd, 0);
    analogWrite(MotorBck, 0);
    //digitalWrite(MotorFwd, HIGH);
    //digitalWrite(MotorBck, HIGH);
  }
}

//Functions///////////////////////////////////////////////////////////////////////////////////////////
void ReadEncoder() { //Function to read the encoder position
  long newM1;
  newM1 = M1.read();
  /*if (newM1 >= (CPT + 1)) {
    newM1 = 0;
    M1.write(0);
    revCount = revCount + 1;
    Serial.print("Rev = ");
    Serial.print(revCount);
    Serial.println();
  }

  if (newM1 <= (-CPT - 1)) {
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
