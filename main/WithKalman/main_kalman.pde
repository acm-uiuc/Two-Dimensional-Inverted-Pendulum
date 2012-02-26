// Released under Creative Commons License
// This code reads gyroscope and accelerometer data from Sparkfun's Razor 6 DOF board
// and processes it using a Kalman Filter algorithm
//
// A lot of code taken from:
// DCM algorithm originally developed by Bill P. and Paul B, in part based on Mahony's work
// Code was ported to ArduIMU by: 
// Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson and Nathan Sindle (SparkFun).
// Version 1.3 (of ArduIMU) updated by Doug Weibel to correct coordinate system, correct pitch/roll drift cancellation, correct yaw drift cancellation and fix minor gps bug.
//
// We no longer use DCM but we use a lot of the same code from their implementation
//
// Connection pinout diagram at
// http://voidbot.net/razor-6dof.html
//
// - by automatik
//
// Kalman Filter and PID code by:
// UIUC Association for Computing Machinery (ACM) Special Interest Group for Robotics (SIGBOT), 2010-2012

#include <Servo.h> 

#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define Gyro_Gain 2.5 //2.5Gyro gain
#define Gyro_Scaled(x) x*((Gyro_Gain*PI)/360)//Return the scaled ADC raw data of the gyro in radians for second
#define G_Dt(x) x*.02 //DT .02 = 20 miliseconds, value used in derivations and integrations

#define ToRad(x) (x*PI)/180.0
#define ToDeg(x) (x*180.0)/PI

#define Kp_ROLLPITCH 0.0075 //.015 Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.000010 //0.000005Pitch&Roll Integrator Gain
#define Kp_YAW .25 //.5Yaw Porportional Gain  
#define Ki_YAW 0.0005 //0.0005Yaw Integrator Gain

/* Matrix sizes */
#define n 4 //state size
#define p 1 //input size
#define m 6 //sensor size


/*Min Speed Filter for Yaw Correction*/
#define SPEEDFILT 1 //1=use min speed filter for yaw drift cancellation, 0=do not use

/*For debugging propurses*/
#define OMEGAA 1 //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift)
#define PRINT_DCM 0 //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 // If 1 will print the analog raw data
#define PRINT_EULER 1 //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_GPS 0

// PID thingers
/*Motor command constants*/
#define MOTOR_RANGE 20
#define MOTOR_CW_LOW 95
#define MOTOR_CW_HIGH MOTOR_CW_LOW+MOTOR_RANGE
#define MOTOR_CCW_HIGH MOTOR_CW_LOW-1
#define MOTOR_CCW_LOW MOTOR_CCW_HIGH-MOTOR_RANGE

// PID Gains
//#define ki 0.0
//#define kd 2.5
//#define kp 1.5
#define setpoint 0

Servo Motor1;
Servo Motor2;

void Actuate(void);
void MotorTest(void);

#define joints 2            //joints refers to number of actuators
#define error_readings 3    //Derivative in PID is based on this many error readings

// j = 0 -> Motor 1
// j = 1 -> Motor 2
// k = 0 -> suffix _2 is the previous state for states
// k = 1 -> suffix _1 is the previous state for states
unsigned short int j;
unsigned short int k;

// Motor Directions
//signed short int dir[2];
float dangle[2];
// PID variables for each Motor
float P[joints], I[joints], D[joints];
float err[joints][error_readings], out[joints], sum[joints];
unsigned int max_output = 200;


// loop counter
int loopcount = 0;

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
float SENSOR_SIGN[]={1,-1,1,1,-1,1}; //{1,1,-1,1,-1,1}Used to change the polarity of the sensors{-1,1,-1,-1,-1,1}
int SensorData[6]; //array that store the 6 ADC filtered data
int SensorData_Offset[6]; //Array that stores the Offset of the gyros

float printtime;
float motortime[2];

int read6, read7, read8, read9;

int long timer_read = 0;
int long timer=0; //general purpose timer 
int long timer24=0; //Second timer used to print values 
int EX[6]; //General purpose array to send information

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};
float errorCourse=180; 
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

unsigned int counter=0;


union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

//uBlox Checksum
byte ck_a=0;
byte ck_b=0;
long iTOW=0; //GPS Millisecond Time of Week
long alt=0; //Height above Ellipsoid 
float speed_3d=0; //Speed (3-D)  (not used)


volatile uint8_t MuxSel=0;
//volatile uint8_t analog_reference = DEFAULT;
volatile int16_t analog_buffer[8];





void test(float value[9],int pos)
{
  Serial.print(convert_to_dec(value[pos]));
}

void setup()
{
  Serial.begin(38400);
  
  analogReference(EXTERNAL);//Using external analog reference
  
  for(int c=0; c<75; c++) // "warm-up ADC
  {
    read_adc_raw();
  }

  for(int y=0; y<=5; y++)
  {
    SensorData_Offset[y]=SensorData[y];
    Serial.println((int)SensorData_Offset[y]);
  }
    SensorData_Offset[5]=SensorData[5]+GRAVITY;
    printtime = millis();
    motortime[0]=millis();
    motortime[1]=millis();
    Motor1.attach(3);
    Motor2.attach(5);
    
      pinMode(6, INPUT);
   pinMode(7, INPUT);
    pinMode(8, INPUT);
     pinMode(9, INPUT);

}

void loop() //Main Loop
{
  roll=(ToDeg(atan2(DCM_Matrix[2][1],DCM_Matrix[2][2])));
  pitch=(ToDeg(asin(DCM_Matrix[2][0])));
  yaw=(ToDeg(atan2(DCM_Matrix[1][0],DCM_Matrix[0][0])));
  
  // print raw gyro data
  dangle[0]=-read_adc(0);
  dangle[1]=-read_adc(1);
  
  if((millis()-timer)>=5)
  {
    timer=millis();
    read_adc_raw(); //ADC Stuff
    //Matrix_update();
    //Normalize();
    //roll_pitch_drift();
    //printdata();
  }
  /*
  if(millis()-printtime>500){
    printtime = millis();
    Serial.print(dangle[0]);
    Serial.print(dangle[1]);
    printdata(); //Send info via serial
  }
  */
  
//out = out^2;
/*  if ((millis() - timer_read) >= 1000) {
    timer_read = millis();

  read6 = digitalRead(6);
  read7 = digitalRead(7);
  read8 = digitalRead(8);
  read9 = digitalRead(9);
  
  
  if (read6 == HIGH) {
    kp ++;
  }
  if (read7 == HIGH) {
    kp --;if(kp<0)kp=0;
  }
  if (read8 == HIGH) {
    kd ++;
  }
  if (read9 == HIGH) {
    kd --;
    if(kd<0)kd=0;
  }
  
  
 // read6 = read7 = read8 = read9 =  0;
  //printgains();

}*/
 

  // actuate y-axis
  j = 0;
  KalmanFilter();
  StateSpaceControl();
  Actuate();
  
  // actuate x-axis
  j = 1;
  KalmanFilter();
  StateSpaceControl();
  Actuate();

  //update which state stores the current state/covariance
  k = (k + 1) % 2;
}


void printgains(void) {
 
  Serial.print("kp = ");
  Serial.print( kp );
  Serial.print("   Kd = ");
  Serial.print(kd);
  Serial.print("   out = ");
  Serial.print(out[0]);
  Serial.print("   out = ");
  Serial.println(out[1]);
  
  /*
   Serial.print("6 = ");
  Serial.print( read6 );
  Serial.print("   Kd = ");
  Serial.print(read7);
  Serial.print("   Ki = ");
  Serial.println(read8);
  */
  
}
