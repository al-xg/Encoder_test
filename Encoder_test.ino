#include <Encoder.h>

//https://www.pjrc.com/teensy/td_libs_Encoder.html
//https://www.pololu.com/product/3081

int led = 13;
int MotorFwd = 10;
int MotorBck = 9;
bool dir=0;
int retRevs=10;
int power = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

Encoder M1(1, 0);
long positionM1  = -999;
int CPT=12;
long revCount=0;
long prevCount=0;



// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  Serial.println("Encoder Test:");
  power = 30;
}

// the loop routine runs over and over again forever:
void loop() {
  long newM1;
  
  newM1 = M1.read();
  if(newM1 >= (CPT+1)){
      newM1=0;
      M1.write(0);
      revCount=revCount+1;
      Serial.print("Rev = ");
      Serial.print(revCount);
      Serial.println();
      }
      
  if(newM1 <= (-CPT-1)){
      newM1=0;
      M1.write(0);
      revCount=revCount-1;
      Serial.print("Rev = ");
      Serial.print(revCount);
      Serial.println();
      }

  if (newM1 != positionM1) {
    /*Serial.print("M1 = ");
    Serial.print(newM1);
    Serial.println();*/
    positionM1 = newM1;
  }

if(revCount >= retRevs) {dir=1; delay(200);}
if(revCount <= -retRevs){dir=0; delay(200);}
 
if(dir==1){
  analogWrite(MotorFwd, 0);
  analogWrite(MotorBck, power);
  } else {
  analogWrite(MotorBck, 0);
  analogWrite(MotorFwd, power);
  }

  // change the power for next time through the loop:
  //power = power + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  //if (power <= 0 || power >= 255) {
   // fadeAmount = -fadeAmount;
  //}

  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset M1 to zero");
    M1.write(0);
  }
  // wait for 30 milliseconds to see the dimming effect
  //delay(30);
}
