


//uses the global variable j to switch between using 1 and 2
void KalmanFilter() 
                  {

if(j == 0)
{
        H = &H_1[0][0];
        H_t = &H_t_1[0][0];
    if(k == 0){
        X_k_1 = &x_1_1[0];
        X_k = &x_1_2[0];
        P_k_1 = &P_1_1[0][0];
        P_k = &P_1_2[0][0];
    }
    else
    {
        X_k_1 = &x_1_2[0];
        X_k = &x_1_1[0];
        P_k_1 = &P_1_2[0][0];
        P_k = &P_1_1[0][0];
    }
}
else
{
        H = &H_2[0][0];
        H_t = &H_t_2[0][0];
    if(k == 0){
        X_k_1 = &x_2_1[0];
        X_k = &x_2_2[0];
        P_k_1 = &P_2_1[0][0];
        P_k = &P_2_2[0][0];
    }
    else
    {
        X_k_1 = &x_2_2[0];
        X_k = &x_2_1[0];
        P_k_1 = &P_2_2[0][0];
        P_k = &P_2_1[0][0];
    }
}
    
Serial.println("Beginning");

// predict
// predicted state estimate
math.MatrixMult((float*)F, (float*)X_k_1, n, n, 1, (float*)temp1);

math.MatrixMult((float*)B, (float*)&u, n, p, 1, (float*)temp0);
//math.MatrixCopy((float*)temp1, n, 1, (float*)X_k_1);
math.MatrixAdd((float*)temp1, (float*)temp0, n, 1, (float*)X_k);
Serial.println("Estimate");
// predicted estimate covariance
math.MatrixMult((float*)F, (float*)P_k_1, n, n, n, (float*)temp2);
math.MatrixMult((float*)temp2, (float*)F_t, n, n, n, (float*)temp3);
math.MatrixAdd((float*)temp3, (float*)Q, n, n, (float*)P_k_1);
Serial.println("Cov");
// update
// innovation or measurement residual

math.MatrixMult((float*)H, (float*)X_k, m, n, 1, (float*)temp4);
math.MatrixSubtract((float*)ProcessedSensorData, (float*)temp4, m, 1, (float*)temp5);

math.MatrixPrint((float*)ProcessedSensorData,m,1,"Sensordata");

Serial.println("Res");

// innovation or residual covariance
math.MatrixMult((float*)H, (float*)P_k, m, n, n, (float*)tempmn); 

math.MatrixPrint((float*)H,m,n,"H");
math.MatrixPrint((float*)P_k,n,n,"P_k");


Serial.println("ResCov");
math.MatrixPrint((float*)H_t,n,m,"H_t");
math.MatrixPrint((float*)tempmn,m,n,"tempmn");

//math.MatrixMult((float*)tempmn, (float*)H_t, m, n, m, (float*)tempm2);

math.MatrixPrint((float*)tempnm,m,m,"tempm2");
/*
math.MatrixAdd((float*)tempm2, (float*)R, m, m, (float*)tempm);
Serial.println("end rescov");

// Optimal Kalman gain
//math.MatrixPrint((float*)tempm,m,m,"tempm");
//math.MatrixInvert((float*)tempm, m);
Serial.println("KGainstart");

math.MatrixMult((float*)P_k_1, (float*)H_t, n, n, m, (float*)tempnm);
math.MatrixMult((float*)tempnm, (float*)tempm, n, m, m, (float*)tempnm2);
Serial.println("KGain");

// updated state estimate
math.MatrixMult((float*)tempnm2, (float*)temp5, n, m, 1, (float*)X_k);
math.MatrixAdd((float*)X_k, (float*)X_k_1, n, 1, (float*)X_k);
Serial.println("State");
//updated estimate covarience
math.MatrixMult((float*)tempnm2, (float*)H, n, m, n, (float*)temp2);

int i, j;
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      temp2[i][j] = -temp2[i][j];
    }
  }
  for(i = 0; i < n; i++){
    temp2[i][i] +=1;
  }
  
math.MatrixMult((float*)temp2, (float*)P_k_1, n, n, n, (float*)P_k);
*/
Serial.println("Ending");
}
