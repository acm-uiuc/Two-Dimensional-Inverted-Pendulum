// PID controller routine
void PID()
{
  // P: proportional control component
  if (err[j][0] > err_max) {
    err[j][0] = err_max; 
  } else if (err[j][0] < err_min) {
    err[j][0] = err_min;
  }		
  P[j] = err[j][0]*kp;
  
  // I: integral control component
  sum[j] += err[j][0];
  if(sum[j] > sum_max) {
    sum[j] = sum_max; 
  } else if(sum[j] < sum_min) {
    sum[j] = sum_min;
  }	
  I[j] = sum[j]*ki;
  
  // D: derivative control component
  //D[j] = (err[j][0] - err[j][2]);
  D[j] = dangle[j];
  if (D[j] > D_max) {
    D[j] = D_max;
  } else if (D[j] < D_min) {
    D[j] = D_min;
  }
  D[j] *= kd;
  
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
  //out[j] = pow(out[j],2);
  
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
     //out[j] = pow(out[j],2);
     // if (out[j] > max_output) { out[j] = max_output; }
      out[j] = map(out[j], 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
      
      Motor1.write(out[j]);
    }
    else if (dir[j] == 1) {
      // actuate counter clockwise
   //   out[j] = pow(out[j],2);
     // if (out[j] > max_output) { out[j] = max_output; }
      out[j] = map(out[j], 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      
      Motor1.write(out[j]);	
    }	
    if(millis()-motortime[j]>250){
      Serial.println(out[j]); 
      motortime[j]=millis();
    }
  }
  // Motor 2
  else if (j == 1) {
    if (dir[j] == -1) {
      // actuate clockwise
   //    out[j] = pow(out[j],2);
     //  if (out[j] > max_output) { out[j] = max_output; }
      out[j] = map(out[j], 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
     
      Motor2.write(out[j]);
    }
    else if (dir[j] == 1) {
      // actuate counter clockwise
    //  out[j] = pow(out[j],2);
   //   if (out[j] > max_output) { out[j] = max_output; }
      out[j] = map(out[j], 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      
      Motor2.write(out[j]);
    }
    if(millis()-motortime[j]>250){
      Serial.println(out[j]); 
      motortime[j]=millis();
    }	 	
  }	
}
