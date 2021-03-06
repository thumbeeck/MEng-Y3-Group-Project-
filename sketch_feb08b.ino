//Eye Mechanism Code
//Adafruit servo driver library installed >>>>> https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
//X-axis joystick pin: A1
//Y-axis joystick pin: A0
//Button pin: 2

 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  520 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

int xval;
int yval;

int xindex;
int yindex;

int srlength;
int irlength;
int lrlength;
int mrlength;
int solength;
int iolength;

int srpulse;
int irpulse;
int lrpulse;
int mrpulse;
int sopulse;
int iopulse;

const int analogInPin = A0;

const int rows = 2;
const int columns = 3;

//insert fulL MATLAB arraya here using MATLAB code:

//XSR = int64(deform_SR)
//writematrix(XSR,'dataSR.dat')

//to produce a '.dat' file, open with TextEdit and copy mapping
//this is so that commas are inserted and the mapping can be coppied into Arduino
//add additional formatting such as {} and ',' between rows

int srarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } }; 
int irarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } };
int lrarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } };
int mrarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } };
int soarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } };
int ioarray[ rows ][ columns ] = { { 1, 2, 3 }, { 4, 5, 6 } };

// Note: 

// x-axis
// Medial    = +ve
// Lateral   = -ve

// y-axis
// Superior  = +ve
// Inferior  = -ve

// z-axis
// Anterior  = +ve
// Posterior = -ve

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");
  pinMode(analogInPin, INPUT);
  pinMode(2, INPUT);
 
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);

}



void loop() {

//map(value, fromLow, fromHigh, toLow, toHigh)
//value: the number to map.
//fromLow: the lower bound of the value’s current range.
//fromHigh: the upper bound of the value’s current range.
//toLow: the lower bound of the value’s target range.
//toHigh: the upper bound of the value’s target range.


  xval = analogRead(A1);   //read from potentiometer
  yval = analogRead(A0);   //read from potentiometer

    //Map an analog value (0 to 1023) to within mechanisms range
    
    //lexpulse = map(xval, 0,1023, 250, 500);
    //leypulse = map(yval, 0,1023, 250, 500);

    xindex = map(xval, 0,1023, 0, (columns-1));
    yindex = map(yval, 0,1023, 0, (rows-1));

    irlength = irarray[yindex][xindex];
    srlength = srarray[yindex][xindex];
    lrlength = lrarray[yindex][xindex];
    mrlength = mrarray[yindex][xindex];
    solength = soarray[yindex][xindex];
    iolength = ioarray[yindex][xindex];
    

    irpulse = map(lrlength, 1,6, 250, 500); //define min and max of array
    srpulse = map(mrlength, 1,6, 250, 500);
    lrpulse = map(lrlength, 1,6, 250, 500);
    mrpulse = map(mrlength, 1,6, 250, 500);
    sopulse = map(lrlength, 1,6, 250, 500);
    iopulse = map(mrlength, 1,6, 250, 500);
    
    //Seriel Monitor allows you to view value for each each muscle as joystick moved
   Serial.print("IR:\t");
   Serial.print(irpulse);
   Serial.print("\t");
   Serial.print("SR:\t");
   Serial.print(srpulse);
   Serial.print("\t");
   Serial.print("LR:\t");
   Serial.print(lrpulse);
   Serial.print("\t");
   Serial.print("MR:\t");
   Serial.print(mrpulse);
   Serial.print("\t");
   Serial.print("SO:\t");
   Serial.print(sopulse);
   Serial.print("\t");
   Serial.print("IO:\t");
   Serial.print(iopulse);
   Serial.println("\t");
   
//The map() function uses integer math. 
//Fractions might get suppressed due to this. 
//If your project requires precise calculations (e.g. voltage accurate to 3 decimal places), 
//please consider avoiding map() and implementing the calculations manually in your code yourself

    
      pwm.setPWM(0, 0, irpulse);
      pwm.setPWM(1, 0, srpulse);
     //pwm.setPWM(2, 0, lrpulse);
     //pwm.setPWM(3, 0, mrpulse);
     //pwm.setPWM(4, 0, sopulse);
     //pwm.setPWM(5, 0, iopulse);
      
//setPWM(channel, on, off)
//channel: The channel that should be updated with the new values (0..15)
//on: The tick (between 0..4095) when the signal should transition from low to high
//off:the tick (between 0..4095) when the signal should transition from high to low

      //e.g.
      //The following example will cause channel 15 to start low, 
      //go high around 25% into the pulse (tick 1024 out of 4096), 
      //transition back to low 75% into the pulse (tick 3072), 
      //and remain low for the last 25% of the pulse:
      //pwm.setPWM(15, 1024, 3072)
      

      
  delay(5);

}
