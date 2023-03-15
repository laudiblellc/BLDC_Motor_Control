
/* 1/5/18
    Servo Controlled Stepper-> DC Motor

*/


//include necessary libraries
//#include <Encoder.h>
//#include <L298N.h>
//#include <DueTimer.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "QuadEncoder.h"
#include <MiniPID.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// define necessary pins
#define STEP_PIN1 3  // REG PIOD 0x01
#define DIR_PIN1 2  // REG PIOD 0x02
#define MOT1_DTY 4
#define MOT1_DIR 5
#define ENA 6
#define quad_A1 1
#define quad_B1 0
#define MOT1_SPD 11
#define EXTRUDE 7
#define RETRACT 8
#define LED 13
#define MAXENC 2147483140

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL); 

//#define MOT_spd1 7
bool debug = false;
bool fakeOut = false;
bool outSwitch = false;
bool enabled = false;
QuadEncoder enc1(1,quad_A1, quad_B1);
QuadEncoder command1(2,STEP_PIN1, DIR_PIN1);
//const unsigned int mask_quad_A1 = digitalPinToBitMask(quad_A1);
//const unsigned int mask_quad_B1 = digitalPinToBitMask(quad_B1);
const unsigned int mask_step = digitalPinToBitMask(STEP_PIN1);
const unsigned int mask_dir = digitalPinToBitMask(DIR_PIN1);
//const double sleepDelay = 120; // sleep after this many seconds.
const double inPos = 5 ; // encoder counts that qualifies as in position.
const double pwmFreq = 36.62109;  // in kHz.X
volatile int outInt1 = 0;
double maxPwm = 4095;//84000 /  pwmFreq*12; // scaling of the pwm clock. also max/min pwm value.
const int dacRes = 4096;
const double anaRes = dacRes / maxPwm / 2; // 50% duty1 cycle for slow movement. 
long secs;

// define PID parameters
int servoLoopTime = 1000;
int filter = 1;//.9;//0.6;
int timeStep = servoLoopTime/filter; // in us. Time between PID loop calculations.
int accTimeStep = 10000; // in us. Time between PID loop accel calculations.
int speedStep = 500; // in us. Time in us between speed calcs. larger number  =  smooth calc. 
volatile int debugTime = 5; // in ms. Time between debug serial checks. 
//int gearRatio = 3;
double KP = 20;//4;//12
double KI = 0.02;//.25;// 0.02
double KD =1000;
double KF =0;//30 ;
#define SPDN 1 //.9;//0.6;
#define ACCN 1 //.9;//0.6;
double KA = 500;//50;//.1 
double maxXYSpeed = 350; //was 500, from printer config in mm/s
double stepDistance = .00488 ; // printer step distance in mm per step. Multiply by countsPerStep for mm per encoder count. 
double countsPerStep = 10 ; // Number of encoder counts per printer step. 
double maxV = maxXYSpeed/10;// /10, 2 *KP* maxXYSpeed  * countsPerStep * timeStep/ stepDistance/10000;//calculate expected counts per PID cycle.
double maxI = 10000;//2*maxV;//10000;
const int maxFollowingError = 8192;// 1.1 * maxV; // Make this just higher than maxV.
const int minPwm = dacRes *0.2;//maxPwm * 0.2 ; actual minimum PWM value
const double minduty=0.91, maxduty=0.99, medduty=0.95; //minduty1 .5
double  absOut1 = 0, absOut2 = 0, scaler=0;
double  duty1= 0.5, duty2=0.5;
const double minPeriod = 1000, maxPeriod = 1000;
double bumpHeight = 1;
double bumpWidth = 30;
double bumpOffset = 6;
double speed1Array[SPDN], speed2Array[SPDN], accel1Array[ACCN], accel2Array[ACCN];

bool sendPulse = true;
#define OUTPUT_MIN -maxPwm
#define OUTPUT_MAX maxPwm
double mics = 0;
// declare variables.
double motPosition1  = 0, motPosition2  = 0;
double reqPosition1 = 0, reqPosition2 = 0;
double steps = 0;
double positionError = 0;
double lastoutput1  = 0;
int lastoutInt1  = 0;
double output1  = 0;
double lastPosition1  = 0;
//double lastPulseTime1 = 1, lastPulseTime2 = 1;
//double accelPulseTime1 = 1, accelPulseTime2 = 1;
double lastCommand1 = 0;
double spd1=0, lastSpeed1=0;
elapsedMicros lastmicros =0;
elapsedMicros speed1Micros =0,  accelMicros=0;
elapsedMillis lastmillis = 0;
double mils = 0;
double pwmCount = 0;
double accel1, lastAccel1, spdOut1, accelOut1;
double calVal1 =0, calVal=0;
// Configure PID routine
MiniPID PIDLoop1= MiniPID(KP, KI, KD, KF, KA);

void setup() {

  enc1.setInitConfig();
  //Optional filter setup
  enc1.EncConfig.filterCount = 0x10;
  enc1.EncConfig.filterSamplePeriod = 100;
  enc1.init();
  command1.setInitConfig();
  command1.EncConfig.decoderWorkMode = 1; // set to step/dir mode
  command1.EncConfig.filterCount = 0x10;
  command1.EncConfig.filterSamplePeriod = 100;
  command1.init();

  //pinMode(DIR_PIN1, INPUT);
  pinMode(ENA, INPUT);
  //pinMode(FERROR, OUTPUT);
 // digitalWrite(FERROR, 0);
  //pinMode(STEP_PIN1, INPUT);
  pinMode(MOT1_SPD, OUTPUT);
  pinMode(MOT1_DIR, OUTPUT);
  pinMode(MOT1_DTY, OUTPUT);
  analogWriteResolution(12);  // can we change for teensy?
  analogWriteFrequency(MOT1_DTY, pwmFreq*1000);  // 50MHz for Teensy
  analogWriteFrequency(MOT1_SPD, pwmFreq*1000);  // 25MHz for enable line
  pinMode(LED, OUTPUT);
  //analogWrite(MOT1_SPD,dacRes * 0.5 );
  //attachInterrupt(STEP_PIN1, readSteps, RISING);  // stepper command pin on interrupt 1 - pin 3

  pwmOut(0);
  PIDLoop1.setMaxIOutput(maxI);
  PIDLoop1.setOutputLimits(maxPwm);
  PIDLoop1.setOutputRampRate(maxV);
  PIDLoop1.setSetpointRange(maxFollowingError);
  PIDLoop1.setOutputFilter(filter);
  PIDLoop1.setPID(KP,KI,KD,KF,KA);
  if (debug){Serial.begin(115200);}
  else{ Serial.begin(9600);}
  //Timer4.attachInterrupt(enPWM).start(1000); // in us. 
  
  //help();
  enabled = false;
  //digitalWrite(MOT1_SPD, false);
  for (int i=0; i< ACCN; i++){
    accel1Array[i] = 0;
  }
  for (int i=0; i< SPDN; i++){
    speed1Array[i] = 0;
  }
}

void runPID(){
  motPosition1 = enc1.read();
  reqPosition1 = command1.read()*countsPerStep;
  if (enabled)
  {   
    if (!fakeOut){
      output1 = PIDLoop1.getOutput(motPosition1, reqPosition1);
      }
    pwmOut(output1);
    digitalWrite(LED,1);
  }
  else{  
    command1.write( 0);
    enc1.write( 0);
    pwmOut(0);
    digitalWrite(LED,0);
  }
}

void loop() {
    // us task loop.
    if (lastmicros >= timeStep) // us tasks
  {
    // Run PID loop
    runPID();
    lastmicros = lastmicros - timeStep;
  }    
   runSpeedCalcs();

  // us task loop.
  if (accelMicros >= accTimeStep) // us tasks
{
  runAccelCalcs();
      accelMicros = accelMicros - accTimeStep;

}
  //  ms task loop
  if (lastmillis >= debugTime) // 100 ms tasks
  {
    // Check for following error!
    if (abs(reqPosition1 - motPosition1)  > maxFollowingError){
    //digitalWrite(FERROR, 1);
    command1.write( 0);
    enc1.write( 0);
    pwmOut(0);
    digitalWrite(LED,0);
    delay(1000);
    CPU_RESTART;
    }
    enabled = !(digitalRead(ENA));
    if (debug){
    if (fakeOut){
      output1 += 1;
      if (output1 > maxPwm/2){
      output1 = -maxPwm/2;
        }
      }
      if ((motPosition1 != reqPosition1)){

        Serial.print(reqPosition1);
        Serial.print("\t");
       // Serial.print(reqPosition2-motPosition2);
      //  Serial.print("\t");
       //Serial.print(PIDLoop1.Doutput);
        //Serial.print("\t");
       Serial.println(motPosition1);
 
        }
      //lastPosition1 = motPosition1;
      //lastPosition2 = motPosition2;
      //digitalWrite(LED,!digitalRead(ENA));
      if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
     }
    lastmillis = 0;
  }
}

double ringBufAcc1(double newTerm){
  double result = 0;
  for (int i = ACCN-1; i>0; i--)
  {
    accel1Array[i]= accel1Array[i-1];
    result += accel1Array[i] / ACCN;
  }
  accel1Array[0] = newTerm;
  result += newTerm / ACCN;  
  return result;
}

double ringBufSpd1(double newTerm){
  double result = 0;
  for (int i = SPDN-1; i>0; i--)
  {
    speed1Array[i]= speed1Array[i-1];
    result += speed1Array[i] / SPDN;
  }
  speed1Array[0] = newTerm;
  result += newTerm / SPDN;  
  return result;
}


void runSpeedCalcs()
{
  reqPosition1 = command1.read()*countsPerStep;

  if (lastCommand1 != reqPosition1){
  spd1 = (reqPosition1 - lastCommand1)*1000/speed1Micros;
  speed1Micros = 1;
  spdOut1 = ringBufSpd1(spd1);
  //spdOut1 = lastSpeed1 + 1/SPDN * (spd1 - lastSpeed1) ;
  if ((spdOut1 < bumpWidth*2) && (spdOut1 > 1/SPDN)&& (bumpHeight != 0)){
  calVal1 = spdOut1+bumpHeight*exp(-pow(((spdOut1/bumpHeight)-bumpOffset/bumpHeight),2));
  }else if ((spdOut1 > -bumpWidth*2) && (spdOut1 < -1/SPDN)&& (bumpHeight != 0)){
  calVal1 = spdOut1-bumpHeight*exp(-pow(((spdOut1/bumpHeight)+bumpOffset/bumpHeight),2));
  }else{calVal1 = spdOut1;}
  lastCommand1 = reqPosition1;
  PIDLoop1.setSpeed(calVal1);
  } 

  if (speed1Micros > 9999){
    spd1 = 0;
  PIDLoop1.setSpeed(0);
  }

}
void runAccelCalcs(){

  calVal=0;
  accel1 = (spdOut1 - lastSpeed1)*10000/accelMicros;
  accelOut1 = ringBufAcc1(accel1);
  if ((accelOut1 < bumpWidth*2) && (accelOut1 > .1/ACCN)&& (bumpHeight != 0)){
    calVal = accelOut1+bumpHeight*exp(-pow(((accelOut1/bumpHeight)-bumpOffset/bumpHeight),2));
  }else if ((accelOut1 > -bumpWidth*2) && (accelOut1 < -.1/ACCN)&& (bumpHeight != 0)){
    calVal = accelOut1-bumpHeight*exp(-pow(((accelOut1/bumpHeight)+bumpOffset/bumpHeight),2));
  }else{calVal = accelOut1;}
  PIDLoop1.setAccel(accelOut1);
  lastSpeed1 = spdOut1; 
 
}

void pwmOut(double out1) {
  absOut1 = abs(out1);
   if (out1 == 0)
  {
    duty1 = 0;//minduty1;
    outInt1 = 0;// analogWrite(MOT1_DTY,0);//REG_PWM_CDTYUPD0 = 0; //reset PWM output1?
  }else{
   if (absOut1 < minPwm) {// clamp to minimum of 20% for PWM out, and change minimum enable period, change enable duty1.
      outInt1 = minPwm;
      duty1 = map(absOut1,0,minPwm,minduty,maxduty);
    }else{ // normal operating range. Vary PWM out, and enable period and duty1. 
    // calculate pulse width and duty1 cycle.
      outInt1 = map(absOut1,minPwm,OUTPUT_MAX,minPwm,maxPwm);
      duty1 = maxduty;//map(absOut,minPwm,output1_MAX,minduty1,maxduty1);
    }
  }
  analogWrite(MOT1_SPD,outInt1);//REG_PWM_CDTYUPD0 = outInt1; // set PWM output1
  analogWrite(MOT1_DTY,dacRes * duty1 );
  // set direction
  if ((out1 < 0)&&(digitalRead(MOT1_DIR)!=0)) {
    digitalWrite(MOT1_DIR, 0);
   }
  else if ((out1 > 0)&&(digitalRead(MOT1_DIR)!=1)) {
    digitalWrite(MOT1_DIR,1);
  }
}

void process_line() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': KP = Serial.parseFloat(); PIDLoop1.setP(KP); PIDLoop1.reset(); break;
    case 'D': KD = Serial.parseFloat(); PIDLoop1.setD(KD); PIDLoop1.reset(); break;
    case 'I': KI = Serial.parseFloat(); PIDLoop1.setI(KI); PIDLoop1.reset(); break;
    case 'F': KF = Serial.parseFloat(); PIDLoop1.setF(KF); PIDLoop1.reset(); break;
    case 'A': KA = Serial.parseFloat(); PIDLoop1.setF(KA); PIDLoop1.reset(); break;
    case 'T': debugTime = Serial.parseFloat(); break;
    case '?': printPos(); break;
    case 'X': steps = steps + Serial.parseInt();  break;
//    case 'W': pulseWidth =  Serial.parseInt();  break;
    //case 'B': PIDLoop1.stop(); PIDLoop1.setBangBang(Serial.parseFloat()); PIDLoop1.run(); break;
    case 'M':  analogWrite(MOT1_SPD, Serial.parseInt()); break;//REG_PWM_CDTYUPD1 = Serial.parseInt();
    case 'Q': Serial.print("P="); Serial.print(KP); Serial.print(" I="); Serial.print(KI);
      Serial.print(" D="); Serial.print(KD); Serial.print(" F="); Serial.print(KF);
      Serial.print(" A="); Serial.println(KA); break;
    case 'H': help(); break;
    }
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F(" PID_output="));  Serial.print(abs(output1) * 0.8 + minPwm); 
  Serial.print(F(" MOT1_SPD output="));  Serial.print(abs(dacRes * duty1));
  Serial.print(F(" Position=")); Serial.print(motPosition1); 
  Serial.print(F(" Target=")); Serial.println(reqPosition1);
  //if(!isHome)  Serial.print(" NOT");  Serial.println(" home");
}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("by misan, slipton"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("? prints out current encoder, output and setpoint values"));
  Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
  Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
  Serial.println(F("Q will print out the current values of P, I and D parameters"));
  Serial.println(F("W will store current values of P, I and D parameters into EEPROM"));
  Serial.println(F("H will print this help message again"));
  Serial.println(F("B sets Bang Bang"));
  Serial.println(F("A sets Accel"));
  Serial.println(F("M sets pwm duty1 cycle"));
  Serial.println(F("L will execute homing\n"));
  Serial.print("duty1 scale: ");
  Serial.println(minPwm);

}
