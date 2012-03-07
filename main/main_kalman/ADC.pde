void read_adc_raw(void)
{
     SensorData[0]= analogRead(3); // Gx     
     SensorData[1]= analogRead(4); // Gy     Maps sensors to correct order 
     SensorData[2]= analogRead(5); // Gz     No longer Very weak low pass filter
     SensorData[3]= analogRead(0); // Ax     there used to be .9 weight on new data 
     SensorData[4]= analogRead(1); // Ay     and .1 weight on the previous data
     SensorData[5]= analogRead(2); // Az      

}

void read_adc_offset(void)
{
    int i = 0;
    for(i = 0; i < 6; i++){
        ProcessedSensorData[i] = (SensorData[i] - SensorData_Offset[i]) * SENSOR_SIGN[i];
    }
}

int read_adc(int select)
{
  return (SensorData[select]-SensorData_Offset[select])*SENSOR_SIGN[select];
}

void printdata(void)//ToDeg(x)
{     
      //Serial.print("!!!");
      #if PRINT_ANALOGS==1
      //Serial.print("AN0:");
      //Serial.print(read_adc(0));
      //Serial.print(",AN1:");
      //Serial.print(read_adc(1));
      //Serial.print(",AN2:");
      //Serial.print(read_adc(2));  
      //Serial.print(",AN3:");
      //Serial.print(read_adc(3));
      //Serial.print (",AN4:");
      //Serial.print(read_adc(4));
      //Serial.print (",AN5:");
      //Serial.print(read_adc(5));
      //Serial.print (",");
      #endif
      #if PRINT_DCM == 1
      //Serial.print ("EX0:");
      //Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      //Serial.print (",EX1:");
      //Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      //Serial.print (",EX2:");
      //Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      //Serial.print (",EX3:");
      //Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      //Serial.print (",EX4:");
      //Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      //Serial.print (",EX5:");
      //Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      //Serial.print (",EX6:");
      //Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      //Serial.print (",EX7:");
      //Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      //Serial.print (",EX8:");
      //Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      //Serial.print (",");
      #endif
      #if PRINT_EULER == 1
      //Serial.print("RLL:");
      //Serial.print(ToDeg(atan2(DCM_Matrix[2][1],DCM_Matrix[2][2])));
      //Serial.print(",PCH:");
      //Serial.print(ToDeg(asin(DCM_Matrix[2][0])));
      //Serial.print(",YAW:");
      //Serial.print(ToDeg(atan2(DCM_Matrix[1][0],DCM_Matrix[0][0])));
      //Serial.print (",");
      #endif
       #if PRINT_GPS == 1
      //Serial.print("LAT:");
      //Serial.print((long)(lat*10000000));
      //Serial.print(",LON:");
      //Serial.print((long)(lon*10000000));
      //Serial.print(",ALT:");
      //Serial.print(alt_MSL);
      //Serial.print(",COG:");
      //Serial.print((ground_course));
      //Serial.print(",SOG:");
      //Serial.print(ground_speed);
      //Serial.print(",FIX:");
      //Serial.print((int)gpsFix);
      #endif
      
      //Serial.println("***"); 
}

long convert_to_dec(float x)
{
  return x*1000000;
}
/////


//
//void Analog_Reference(uint8_t mode)
//{
//	analog_reference = mode;
//}
//ADC interrupt vector, this piece of
//is executed everytime a convertion is done. 
//ISR(ADC_vect)
//{
//  volatile uint8_t low, high;
//  low = ADCL;
//  high = ADCH;
//  analog_buffer[MuxSel]=(high << 8) | low;
//  MuxSel++;
//  if(MuxSel >=8) MuxSel=0;
//  ADMUX = (analog_reference << 6) | (MuxSel & 0x07);
  // start the conversion
//  ADCSRA|= (1<<ADSC);
//}

