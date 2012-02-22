// Released under Creative Commons License
// This code reads gyroscope and accelerometer data from Sparkfun's Razor 6 DOF board
// and process it with Direction Cosine Matrix algorithm (DCM)
//
// DCM algorithm originally developed by Bill P. and Paul B, in part based on Mahony's work
// Code was ported to ArduIMU by: 
// Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson and Nathan Sindle (SparkFun).
// Version 1.3 (of ArduIMU) updated by Doug Weibel to correct coordinate system, correct pitch/roll drift cancellation, correct yaw drift cancellation and fix minor gps bug.
//
// This code is DCM algorithm adopted for use with Razor board
// code is slightly modified, mainly it does not use innteruprs (in future - I just wanted to make sure it works first)
// Also uncomment portion of the code in the main loop if you want to use GPS code
// Connection pinout diagram at
// http://voidbot.net/razor-6dof.html
//
// - by automatik

#include <Servo.h> 

#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define Gyro_Gain 2.5 //2.5Gyro gain
#define Gyro_Scaled(x) x*((Gyro_Gain*PI)/360)//Return the scaled ADC raw data of the gyro in radians for second
#define G_Dt(x) x*.02 //DT .02 = 20 miliseconds, value used in derivations and integrations

#define ToRad(x) (x*PI)/180.0
#define ToDeg(x) (x*180.0)/PI

#define Kp_ROLLPITCH 0.015 //.015 Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.000010 //0.000005Pitch&Roll Integrator Gain
#define Kp_YAW .5 //.5Yaw Porportional Gain  
#define Ki_YAW 0.0005 //0.0005Yaw Integrator Gain

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
#define MOTOR_RANGE 25
#define MOTOR_CW_LOW 95
#define MOTOR_CW_HIGH MOTOR_CW_LOW+MOTOR_RANGE
#define MOTOR_CCW_HIGH MOTOR_CW_LOW-1
#define MOTOR_CCW_LOW MOTOR_CCW_HIGH-MOTOR_RANGE

// PID Gains
#define ki 0.0
#define kd 6.5
#define kp 4.5
#define setpoint 0

float roll;
float pitch;
float yaw;

Servo Motor1;
Servo Motor2;

void Actuate(void);
void PID(void);
void MotorTest(void);

#define joints 2            //joints refers to number of actuators
#define error_readings 3    //Derivative in PID is based on this many error readings

// j = 0 -> Motor 1
// j = 1 -> Motor 2
unsigned short int j;

// Motor Directions
signed short int dir[2];
float dangle[2];
// PID variables for each Motor
float P[joints], I[joints], D[joints];
float err[joints][error_readings], out[joints], sum[joints];
unsigned int max_output = 200;

// PID limits
int sum_max = 10000;
int sum_min = -10000;
int D_max = 300;
int D_min = -300;
int err_max = 10000; 
int err_min = -10000;

// loop counter
int loopcount = 0;

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
float SENSOR_SIGN[]={1,-1,1,1,-1,1}; //{1,1,-1,1,-1,1}Used to change the polarity of the sensors{-1,1,-1,-1,-1,1}

float printtime;
float motortime[2];

int long timer=0; //general porpuse timer 
int long timer24=0; //Second timer used to print values 
int AN[6]; //array that store the 6 ADC filtered data
int AN_OFFSET[6]; //Array that stores the Offset of the gyros
int EX[6]; //General porpuse array to send information

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};
float errorCourse=180; 
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
 
//GPS 

//GPS stuff please read SiRF-Binary-Protocol-Reference-Manual page 87 for more information
union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

/*Flight GPS variables*/
int gpsFix=1; //This variable store the status of the GPS
float lat=0; // store the Latitude from the gps
float lon=0;// Store guess what?
float alt_MSL=0; //This is the alt.
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=90;//This is the runaway direction of you "plane" in degrees
float climb_rate=0; //This is the velocity you plane will impact the ground (in case of being negative) in meters for seconds
char data_update_event=0; 

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
    AN_OFFSET[y]=AN[y];
    Serial.println((int)AN_OFFSET[y]);
  }
    AN_OFFSET[5]=AN[5]+GRAVITY;
    printtime = millis();
    motortime[0]=millis();
    motortime[1]=millis();
    Motor1.attach(5);
    Motor2.attach(3);
}

void loop() //Main Loop
{
  roll=(ToDeg(atan2(DCM_Matrix[2][1],DCM_Matrix[2][2])));
  pitch=(ToDeg(asin(DCM_Matrix[2][0])));
  yaw=(ToDeg(atan2(DCM_Matrix[1][0],DCM_Matrix[0][0])));
  
  // print raw gyro data
  dangle[1]=-read_adc(0);
  dangle[0]=-read_adc(1);
  
  if((millis()-timer)>=5)
  {
    timer=millis();
    read_adc_raw(); //ADC Stuff
    Matrix_update();
    Normalize();
    roll_pitch_drift();
  }
  /*
  if(millis()-printtime>500){
    printtime = millis();
    Serial.print(dangle[0]);
    Serial.print(dangle[1]);
    printdata(); //Send info via serial
  }
  */
  // actuate y-axis
  j = 0;
  err[j][0] = setpoint - roll*10;
  PID();
  Actuate();
  
  // actuate x-axis
  j = 1;
  err[j][0] = setpoint - pitch*10;  
  PID();
  Actuate();
}
// PID controller routine
void PID()
{
  // P: proportional control component
  if (err[j][0] > err_max) {
    err[j][0] = err_max; 
  } else if (err[j][0] < err_min) {
    err[j][0] = err_min;
  }		
  P[j] = err[j][0];
  
  // I: integral control component
  sum[j] += err[j][0];
  if(sum[j] > sum_max) {
    sum[j] = sum_max; 
  } else if(sum[j] < sum_min) {
    sum[j] = sum_min;
  }	
  I[j] = sum[j];
  
  // D: derivative control component
  //D[j] = (err[j][0] - err[j][2]);
  D[j] = dangle[j];
  if (D[j] > D_max) {
    D[j] = D_max;
  } else if (D[j] < D_min) {
    D[j] = D_min;
  }
  
  
  // sum the control components amplified by their gains
  out[j] = kp * P[j] + ki * I[j] + kd * D[j];
  
  // determine motor direction
  if (out[j] >= 0) {
    dir[j] = 1;
  }else{
    dir[j] = -1;
  }
  // Saturate output if greater than the max allowed
  // ???
  if (abs(out[j]) > max_output) {
    out[j] = max_output;  
  }
  out[j] = abs(out[j]);
  
  // Shift errors
  // ???
  unsigned int i;
  for (i = 2; i > 0; i --) {
    err[j][i] = err[j][i - 1];
  }
  err[j][0] = 0;
}
// Actuate the motors based on out[]
// out[] is the result of PID()
void Actuate(void) {
  // Motor1
  if (j == 0) {
    if (dir[j] == -1) {
      // actuate clockwise
      out[j] = map(out[j], 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
      Motor1.write(out[j]);
    }
    else if (dir[j] == 1) {
      // actuate counter clockwise
      out[j] = map(out[j], 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      Motor1.write(out[j]);	
    }	
    if(millis()-motortime[j]>250){
      //Serial.println(out[j]); 
      motortime[j]=millis();
    }
  }
  // Motor 2
  else if (j == 1) {
    if (dir[j] == -1) {
      // actuate clockwise
      out[j] = map(out[j], 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
      Motor2.write(out[j]);
    }
    else if (dir[j] == 1) {
      // actuate counter clockwise
      out[j] = map(out[j], 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      Motor2.write(out[j]);
    }
    if(millis()-motortime[j]>250){
      //Serial.println(out[j]); 
      motortime[j]=millis();
    }	 	
  }	
}
