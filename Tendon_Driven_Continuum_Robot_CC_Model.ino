
#include <Arduino.h>
#include "A4988.h" 

#define MOTOR_STEPS 200 // using a 200-step motor (most common)

//// configure the pins connected
// Stepper Motor 1
#define DIR 23
#define STEP 22
#define MS1 46
#define MS2 47
#define MS3 48

// Stepper Motor 2
#define DIR2 28
#define STEP2 29
#define MS1_2 49
#define MS2_2 50
#define MS3_2 51

// Stepper Motor 3
#define DIR3 31
#define STEP3 30
#define MS1_3 40
#define MS2_3 41
#define MS3_3 42

// Stepper Motor 4
#define DIR4 37
#define STEP4 36
#define MS1_4 43
#define MS2_4 44
#define MS3_4 45

// Rotary encoder 1
const int PinCLK=3; // Generating interrupts using CLK signal
const int PinDT=24; // Reading DT signal
const int PinSW=25; // Reading Push Button switch

// Rotary encoder 2
const int PinCLK2=2; // Generating interrupts using CLK signal
const int PinDT2=26; // Reading DT signal
const int PinSW2=27; // Reading Push Button switch

// Rotary encoder 3
const int PinCLK3=18; // Generating interrupts using CLK signal
const int PinDT3=32; // Reading DT signal
const int PinSW3=33; // Reading Push Button switch

// Rotary encoder 4
const int PinCLK4=19; // Generating interrupts using CLK signal
const int PinDT4=34; // Reading DT signal
const int PinSW4=35; // Reading Push Button switch

////  Configrations and setups
A4988 stepper1(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);
A4988 stepper2(MOTOR_STEPS, DIR2, STEP2, MS1_2, MS2_2, MS3_2);
A4988 stepper3(MOTOR_STEPS, DIR3, STEP3, MS1_3, MS2_3, MS3_3);
A4988 stepper4(MOTOR_STEPS, DIR4, STEP4, MS1_4, MS2_4, MS3_4);


// Rotary endoder 1 setup
volatile boolean TurnDetected; // need volatile for Interrupts
volatile boolean rotationdirection; // CW or CCW rotation
const int interruptpin1=PinCLK ; // Interrupt pin1 number
int RotaryPosition=0; // To store Stepper Motor Position
int PrevPosition; // Previous Rotary position Value to check accuracy
float AnglesToRotate=0; // How much angle to move Stepper
int encodata;

// Rotary endoder 2 setup
volatile boolean TurnDetected2; // need volatile for Interrupts
volatile boolean rotationdirection2; // CW or CCW rotation
const int interruptpin2=PinCLK2 ; // Interrupt pin1 number
int RotaryPosition2=0; // To store Stepper Motor Position
int PrevPosition2; // Previous Rotary position Value to check accuracy
float AnglesToRotate2=0; // How much angle to move Stepper
int encodata2;

// Rotary endoder 3 setup
volatile boolean TurnDetected3; // need volatile for Interrupts
volatile boolean rotationdirection3; // CW or CCW rotation
const int interruptpin3=PinCLK3 ; // Interrupt pin1 number
int RotaryPosition3=0; // To store Stepper Motor Position
int PrevPosition3; // Previous Rotary position Value to check accuracy
float AnglesToRotate3=0; // How much angle to move Stepper
int encodata3;

// Rotary endoder 4 setup
volatile boolean TurnDetected4; // need volatile for Interrupts
volatile boolean rotationdirection4; // CW or CCW rotation
const int interruptpin4=PinCLK4 ; // Interrupt pin1 number
int RotaryPosition4=0; // To store Stepper Motor Position
int PrevPosition4; // Previous Rotary position Value to check accuracy
float AnglesToRotate4=0; // How much angle to move Stepper
int encodata4;


//// Interrupt routine runs if CLK goes from HIGH to LOW for each encoder
//[1]
void isr () {
 delay(4); // delay for Debouncing
 encodata=digitalRead(PinCLK);
 if (digitalRead(PinCLK))
 rotationdirection= digitalRead(PinDT);
 else
 rotationdirection= !digitalRead(PinDT);
 TurnDetected = true;
}

//[2]
void isr2 () {
 delay(4);
 encodata2=digitalRead(PinCLK2);
 if (digitalRead(PinCLK2))
 rotationdirection2= digitalRead(PinDT2);
 else
 rotationdirection2= !digitalRead(PinDT2);
 TurnDetected2 = true;
}
//[3]
void isr3 () {
 delay(4); // delay for Debouncing
 encodata3=digitalRead(PinCLK3);
 if (digitalRead(PinCLK3))
 rotationdirection3= digitalRead(PinDT3);
 else
 rotationdirection3= !digitalRead(PinDT3);
 TurnDetected3 = true;
}
//[4]
void isr4 () {
 delay(4); // delay for Debouncing
 encodata4=digitalRead(PinCLK4);
 if (digitalRead(PinCLK4))
 rotationdirection4= digitalRead(PinDT4);
 else
 rotationdirection4= !digitalRead(PinDT4);
 TurnDetected4 = true;
}



float i=100; // angle increment index for each encoder
float alpha=1.00765 ; // 1.00765,n1.1085 Segment 1 Actuation Compensation Factor
float Rx2; // Segment 2 x rotation angle
float Ry2; // Segment 2 y rotation angle
float Rx1; // Segment 1 x rotation angle
float Ry1; // Segment 1 y rotation angle
float Ay2;
float Ax2;
float Ay1;
float Ax1;


void setup() {
 // Set target motor RPM to 1RPM and microstepping to 1 (full step mode)
 stepper1.begin(180, 16);
 
//Rotary encoder 1 setup
pinMode(PinCLK,INPUT);
pinMode(PinDT,INPUT); 
pinMode(PinSW,INPUT);
digitalWrite(PinSW, HIGH); // Pull-Up resistor for switch

// Rotary encoder 2 setup
stepper2.begin(180, 16);
pinMode(PinCLK2,INPUT);
pinMode(PinDT2,INPUT); 
pinMode(PinSW2,INPUT);
digitalWrite(PinSW2, HIGH); // Pull-Up resistor for switch

// Rotary encoder 3 setup
stepper3.begin(180, 16);
pinMode(PinCLK3,INPUT);
pinMode(PinDT3,INPUT); 
pinMode(PinSW3,INPUT);
digitalWrite(PinSW3, HIGH); // Pull-Up resistor for switch


// Rotary encoder 4 setup
stepper4.begin(180, 16);
pinMode(PinCLK4,INPUT);
pinMode(PinDT4,INPUT); 
pinMode(PinSW4,INPUT);
digitalWrite(PinSW4, HIGH); // Pull-Up resistor for switch

attachInterrupt (digitalPinToInterrupt(PinCLK),isr,CHANGE); // interrupt 0 always 
attachInterrupt (digitalPinToInterrupt(PinCLK2),isr2,CHANGE); // interrupt 0 always 
attachInterrupt (digitalPinToInterrupt(PinCLK3),isr3,CHANGE); // interrupt 3 setup 
attachInterrupt (digitalPinToInterrupt(PinCLK4),isr4,CHANGE); // interrupt 3 setup 

Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}



void loop() {
 if (!(digitalRead(PinSW))) { // check if button is pressed
 if (RotaryPosition == 0) { // check if button was already pressed
 } 
 else {
 stepper1.move(-(RotaryPosition*i));
 RotaryPosition=0; // Reset position to ZERO
 }
 }
 
if (TurnDetected3 ||TurnDetected || TurnDetected2|| TurnDetected4) {

//encoder 1 is turned 
if (TurnDetected) {
 PrevPosition = RotaryPosition; // Save previous position in variable
 if (rotationdirection) {
 RotaryPosition=RotaryPosition-1;} // decrase Position by 1
 else {
 RotaryPosition=RotaryPosition+1;} // increase Position by 1
 TurnDetected = false; // do NOT repeat IF loop until new rotation detected
 // Which direction to move Stepper motor
 if ((RotaryPosition + 1) == PrevPosition) { // Move motor CCW
 
 AnglesToRotate=-i*1.8/16;
 // stepper1.rotate(AnglesToRotate);
 }
 else if ((PrevPosition + 1) == RotaryPosition) { // Move motor CW
 AnglesToRotate=i*1.8/16;
 stepper1.rotate(AnglesToRotate);
 }
 else {AnglesToRotate=0 ;} 
// Serial print encoder
 Serial.print("encodata1:");
Serial.println(encodata,DEC);
 }



//encoder 2 is turned  
if (TurnDetected2) {
 PrevPosition2 = RotaryPosition2; // Save previous position in variable
 if (rotationdirection2) {
 RotaryPosition2=RotaryPosition2-1;} // decrase Position by 1
 else {
 RotaryPosition2=RotaryPosition2+1;} // increase Position by 1
 TurnDetected2 = false; // do NOT repeat IF loop until new rotation detected
 // Which direction to move Stepper motor
 if ((RotaryPosition2 + 1) == PrevPosition2) { // Move motor CCW
 
 AnglesToRotate2=-i*1.8/16;
 stepper2.rotate(AnglesToRotate2);
 }
 else if ((PrevPosition2 + 1) == RotaryPosition2) { // Move motor CW
 AnglesToRotate2=i*1.8/16;
 stepper2.rotate(AnglesToRotate2);
 }
 else {AnglesToRotate2=0 ;}
// Serial print encoder
 
Serial.print("encodata2:");
Serial.println(encodata2,DEC);
 }



//encoder 3 is turned 
if (TurnDetected3) {
PrevPosition3 = RotaryPosition3; // Save previous position in variable
 if (rotationdirection3) {
 RotaryPosition3=RotaryPosition3-1;} // decrase Position by 1
 else {
 RotaryPosition3=RotaryPosition3+1;} // increase Position by 1
 TurnDetected3 = false; // do NOT repeat IF loop until new rotation detected
 // Which direction to move Stepper motor
 if ((RotaryPosition3 + 1) == PrevPosition3) { // Move motor CCW
 AnglesToRotate3=-i*1.8/16;
 stepper3.rotate(AnglesToRotate3);
 }
 else if ((PrevPosition3 + 1) == RotaryPosition3) { // Move motor CW
 AnglesToRotate3=i*1.8/16;
 stepper3.rotate(AnglesToRotate3);
 }
 else {AnglesToRotate3=0 ;}
// Serial print encoder 
Serial.print("encodata3:");
Serial.println(encodata3,DEC); 
}



//encoder 4 is turned  
if (TurnDetected4) {
PrevPosition4 = RotaryPosition4; // Save previous position in variable
 if (rotationdirection4) {
 RotaryPosition4=RotaryPosition4-1;} // decrase Position by 1
 else {
 RotaryPosition4=RotaryPosition4+1;} // increase Position by 1
 TurnDetected4 = false; // do NOT repeat IF loop until new rotation detected
 // Which direction to move Stepper motor
 if ((RotaryPosition4 + 1) == PrevPosition4) { // Move motor CCW
 
 AnglesToRotate4=-i*1.8/16;
 stepper4.rotate(AnglesToRotate4);
 }
 else if ((PrevPosition4 + 1) == RotaryPosition4) { // Move motor CW
 AnglesToRotate4=i*1.8/16;
 stepper4.rotate(AnglesToRotate4);
 }
 else {AnglesToRotate4=0 ;}
 
// Serial print encoder
Serial.print("encodata4:");
Serial.println(encodata4,DEC); 
}

//// displacemments of 2 antagonist tendons according to the constant curvature assumption (model)
Rx2=(AnglesToRotate3+AnglesToRotate*0.866025+AnglesToRotate2*0.5);
Ry2=(AnglesToRotate4+AnglesToRotate2*0.866025-0.5*AnglesToRotate);
Ry1=AnglesToRotate2-alpha*0.5*AnglesToRotate4*0.866025-(alpha)*0.5*AnglesToRotate3*0.5;
Rx1=AnglesToRotate-alpha*0.5*AnglesToRotate3*0.866025+(alpha)*0.5*AnglesToRotate4*0.5;


int j=6;
Ay2=Ry2/j;
Ax2=Rx2/j;
Ay1=Ry1/j;
Ax1=Rx1/j;

for(int s=0; s<j; s++)
{stepper1.rotate(Ay2);
stepper2.rotate(Ax2);
stepper3.rotate(Ay1);
stepper4.rotate(Ax1);  
}

// reset angles to rotate to zero
AnglesToRotate=0; 
AnglesToRotate2=0;
AnglesToRotate3=0;
AnglesToRotate4=0;
}
}
