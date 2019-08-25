//MegunoLINK//////////////////////////////////////////////////////////////////////////////////////
#include <MegunoLink.h>
#include <CommandHandler.h>
#include <TCPCommandHandler.h>
#include <ArduinoTimer.h>
#include <CircularBuffer.h>
//#include <EEPROMStore.h>
#include <Filter.h>

TimePlot MyPlot;  // The plot we are sending data to.
bool plot = 0;    // Turns on or off the PID plotting

//Keyboard input///////////////////////////////////////////////////////////////////////////////////

//https://forum.arduino.cc/index.php?topic=396450
//https://arduining.com/2015/05/04/serial-control-of-arduino-led/
//https://www.oreilly.com/library/view/arduino-cookbook/9781449399368/ch04.html (haven't actually read this one yet)

char rxChar = 0;                 // RXcHAR holds the received command.
const byte numChars = 6;         // Number of characters to consider for a command
char receivedChars[numChars];
boolean newData = false;         // Flag to trigger updating variables once new chars received

//Timing//////////////////////////////////////////////////////////////////////////////////////////

long previousMillis = 0;        // Stores last time MegunoLINK was updated
long interval = 50;             // Interval at which to update (milliseconds)

//Functions////////////////////////////////////////////////////////////////////////////////////////

void plotThings() {             // Reports to MegunoLINK at the defined time interval
  if (plot == 1) {

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;           // Save the last time this statement was entered

      MyPlot.SendData("Setpoint", Setpoint);
      MyPlot.SendData("Input", Input);
      MyPlot.SendData("Output", Output);

      /*Serial.print("   Setpoint: ");
        Serial.print(Setpoint);
        Serial.print("   Input: ");
        Serial.print(Input);
        Serial.print("   Output: ");
        Serial.print(Output);
        Serial.print("   Power: ");
        Serial.println(power);*/
    }
  }
}

void GetSerialCommands() {                      // Function to read serial and determine cases
  if (Serial.available() > 0) {                 // Check receive buffer.
    rxChar = Serial.read();                     // Save character received.
    Serial.flush();                             // Clear receive buffer.

    switch (rxChar) {

      case 'x':
      case 'X':                                 // If received 'x' or 'X':
        if (StopStart == 0) {
          StopStart = 1;
          Serial.println("Motor turned On.");
        }
        else
        { StopStart = 0;
          Serial.println("Motor turned Off.");
        }
        break;

      case 'c':
      case 'C':                                  // If received  'c' or 'C':
        if (dir == 1) {                          // Read dir status.
          Serial.println("Direction: CCW");
          dir = 0;
        }
        else {
          Serial.println("Direction: CW");
          dir = 1;
        }
        break;

      case 'w':
      case 'W':                                  // If received  'w' or 'W':
        if (plot == 1) {                         // Read plotting status.
          Serial.println("Plotting off.");
          plot = 0;
        }
        else {
          Serial.println("Plotting on.");
          plot = 1;
        }
        break;

      case '?':                       // If received a ?:
        printHelp();                  // print the command list.
        break;

      case 'q':
      case 'Q':                       // If received a 'q' or 'Q':
        recvWithEndMarker();          // Extract value
        checkPositionChars();         // Check the position value and update variable
        break;

      case 'p':
      case 'P':                       // If received a 'p' or 'P':
        recvWithEndMarker();          // Extract value
        checkKpChars();               // Check the Kp value and update variable
        break;

      case 'i':
      case 'I':                       // If received a 'i' or 'I':
        recvWithEndMarker();          // Extract value
        checkKiChars();               // Check the Ki value and update variable
        break;

      case 'd':
      case 'D':                       // If received a 'd' or 'D':
        recvWithEndMarker();          // Extract value
        checkKdChars();               // Check the Kd value and update variable
        break;

      case '\n':
        break;

      default:

        Serial.print("'");
        Serial.print((char)rxChar);
        Serial.println("' is not a valid command!");
    }
  }
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';

  while (Serial.available() > 0 && newData == false) {
    rxChar = Serial.read();

    if (rxChar != endMarker) {
      receivedChars[ndx] = rxChar;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      if (*receivedChars == '\0' || receivedChars == NULL) {
        Serial.println("Empty command ignored.");
        newData = false;
      } else {
        newData = true;
      }
    }
  }
}

void checkPositionChars() {
  if (newData == true) {
    AngleCommand = atol(receivedChars);
    PositionCommand = AngleCommand * (EncToAngle / 1000000);
    Serial.print("New angle command: ");
    Serial.print(AngleCommand);
    Serial.println(" deg");
    newData = false;
  }
}

void checkKpChars() {
  if (newData == true) {
    Kp = atof(receivedChars);
    M1PID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Kp value: ");
    Serial.println(Kp);
    newData = false;
  }
}

void checkKiChars() {
  if (newData == true) {
    Ki = atof(receivedChars);
    M1PID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Ki value: ");
    Serial.println(Ki);
    newData = false;
  }
}

void checkKdChars() {
  if (newData == true) {
    Kd = atof(receivedChars);
    M1PID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Kd value: ");
    Serial.println(Kd);
    newData = false;
  }
}

void printHelp(void) {   // Function to print the command list
  Serial.println();
  Serial.println("--- Command list: ---");
  Serial.println("? -> Print this HELP");
  Serial.println("x -> Toggle motor On/Off");
  Serial.println("c -> Toggle CW/CCW");
  Serial.println("w -> Toggle PID plotting on/off");
  Serial.println("p'xxx' -> Set Kp");
  Serial.println("i'xxx' -> Set Ki");
  Serial.println("d'xxx' -> Set Kd");
  Serial.println("q'xxxxx' -> Set angle to move to in degrees");
  Serial.println();
  Serial.println("--- Current Variables: ---");
  Serial.print("Kp=");
  Serial.print(Kp);
  Serial.print(" Ki=");
  Serial.print(Ki);
  Serial.print(" Kd=");
  Serial.println(Kd);
  Serial.print("Angle Command: ");
  Serial.print(AngleCommand);
  Serial.println(" deg");
  Serial.println();
}
