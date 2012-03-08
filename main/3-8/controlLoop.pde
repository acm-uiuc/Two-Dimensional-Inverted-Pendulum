void StateSpaceControl()
{
    //multiply the k vector with the x vector to find control effort
    if(j == 0){
        if(k == 0){
            math.MatrixMult((float*)x_1_2, (float*)kvector, 1, n, 1, (float*)&u); 
        }
        else{
            math.MatrixMult((float*)x_1_1, (float*)kvector, 1, n, 1, (float*)&u); 
        }
    }
    else{
        if(k == 0){
            math.MatrixMult((float*)x_2_2, (float*)kvector, 1, n, 1, (float*)&u); 
        }
        else{
            math.MatrixMult((float*)x_2_1, (float*)kvector, 1, n, 1, (float*)&u); 
        }
    }
  
  // determine motor direction
  if (u >= 0) {
    dir = 1;
  }else{
    dir = -1;
  }
  // Saturate output if greater than the max allowed
  u = abs(u);
  if (u > max_output) {
    u = max_output;  
  }
  
}

// Actuate the motors based on u
// u is the result of StateSpaceControl()
void Actuate(void) {
  // Motor1
  if (j == 0) {
    if (dir == -1) {
      // actuate clockwise
      u = map(u, 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
      
      Motor1.write(u);
    }
    else if (dir == 1) {
      // actuate counter clockwise
      u = map(u, 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      
      Motor1.write(u);	
    }	
    if((millis() - motortime[j])>250){
      Serial.println(max_output); 
      motortime[j]=millis();
    }
  }
  // Motor 2
  else if (j == 1) {
    if (dir == -1) {
      // actuate clockwise
      u = map(u, 0, max_output, MOTOR_CW_LOW, MOTOR_CW_HIGH);
     
      Motor2.write(u);
    }
    else if (dir == 1) {
      // actuate counter clockwise
      u = map(u, 0, max_output, MOTOR_CCW_HIGH, MOTOR_CCW_LOW);
      
      Motor2.write(u);
    }
    if(millis()-motortime[j]>250){
      Serial.println(max_output); 
      motortime[j]=millis();
    }	 	
  }	
}

