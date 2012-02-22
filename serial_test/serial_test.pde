int val = 0;                    //value of individual accelerometer or gyroscope sensor
unsigned long timer=0;          //timer
unsigned long  delta_t;         //delta time or how long it takes to execute data acquisition 

void setup()

{
  analogReference(EXTERNAL);     //using external analog ref of 3.3V for ADC scaling
  Serial.begin(9600);          //setup serial
  DDRC = B00000000;              //make all analog ports as inputs - just in case....
  
  delay (100); //dealy just in case - to get things stabilized if need be....
 
  Serial.println("t[ms] \t gy \t gx \t gz \t az \t ay \t ax "); //print data header

 timer=millis(); 
}

void loop()
{
  delta_t = millis() - timer; // calculate time through loop i.e. acq. rate
  timer=millis();          // reset timer
  Serial.print(delta_t);
  Serial.print ("\t"); 
     
  for (long i=0; i<6; i++) //read gyroscope and accelerometer sensor data
 { 
  val = analogRead(i);    // read the input pin
  Serial.print(val);      //print data
  Serial.print ("\t");
 }  

  Serial.println("");
  //delay(16);             //loop delay; loop executed at ~ 50Hz or 20ms
}
