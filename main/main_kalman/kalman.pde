/*
  old signature
                  float* F, // n*n
                  float* F_t, // t
                  float* B, // n*p
                  float* u, // p*1   -> can change when done using
                  float* H, // m*n array
                  float* H_t, // transpose of H
                  float* Q, // n*n
                  float* R, // m*m
                  float* Z_k, // m*1 array -- sensor measurement    -> can change
                  float* P_k_1, // n*n
                  float* X_k_1, // n*1
                  float* P_k, // n*n
                  float* X_k

MODEL PARAMETERS BELOW:

\dot{x} = A x + B u
y = C x + D u


x_k = x_k1 + Pkk1 * Htk * Sk^-1 * z

S^k-1 l = z
*/





//covariance matrices
float P_1_1[n][n]={
{
    1,0,0,0  }
,{
    0,1,0,0  }
,{
    0,0,1,0  }
,{
    0,0,0,1  }
};

float P_1_2[n][n]={
{
    1,0,0,0  }
,{
    0,1,0,0  }
,{
    0,0,1,0  }
,{
    0,0,0,1  }
};

float P_2_1[n][n]={
{
    1,0,0,0  }
,{
    0,1,0,0  }
,{
    0,0,1,0  }
,{
    0,0,0,1  }
};

float P_2_2[n][n]={
{
    1,0,0,0  }
,{
    0,1,0,0  }
,{
    0,0,1,0  }
,{
    0,0,0,1  }
};


//kalman filter settings
float F[n][n]={
{
    0,1,0,0  }
,{
    L_3 * k_1,-L_3 * k_2, L_1 - L_3 * k_3, -L_3 * k_4  }
,{
    0,0,0,1  }
,{
    -L_4 * k_1, -L_4 * k_2, L_2 - L_4 * k_3, -L_4 * k_4  }
};

float F_t[n][n]={
{
    0,L_3 * k_1,0,-L_4 * k_1  }
,{
    1,-L_3 * k_2, 0,-L_4 * k_2   }
,{
    0,L_1 - L_3 * k_3,0,L_2 - L_4 * k_3  }
,{
    0, -L_3 * k_4, 1, -L_4 * k_4  }
};

float B[n][p]={
{
    0  }
,{
    L_3  }
,{
    0  }
,{
    L_4  }
};

float H_1[m][n]={
{
    0,0,0,0  }
,{
    0,L_5,0,-L_5  }
,{
    0,0,0,0  }
,{
    0,0,0,0  }
,{
    0,0,0,0  }
,{
    0,0,GRAVITY,0  }
};


float H_2[m][n]={
{
    0,L_6,0,-L_6  }
,{
    0,0,0,0  }
,{
    0,0,0,0  }
,{
    0,0,0,0  }
,{
    0,0,0,0  }
,{
    0,0,GRAVITY,0  }
};

float H_t_1[n][m]={
{
    0,0,0,0,0,0  }
,{
    0,L_5,0,0,0,0  }
,{
    0,0,0,0,0,GRAVITY  }
,{
    0,-L_5,0,0,0,0  }
};


float H_t_2[n][m]={
{
    0,0,0,0,0,0  }
,{
    L_6,0,0,0,0,0  }
,{
    0,0,0,0,0,GRAVITY  }
,{
    -L_6,0,0,0,0,0  }
};





//uses the global variable j to switch between using 1 and 2
void KalmanFilter() 
                  {

float temp1 [n][1];

float temp2 [n][n];
float temp3[n][n];

float temp4[m][1];

float tempmn[m][n];
float tempnm[n][m];
float tempnm2 [n][m];
float tempm [m][m];
float tempm2 [m][m];

float* X_k_1;
float* X_k;
float* P_k_1;
float* P_k;
float* H;
float* H_t;

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
    
//Serial.println("Beginning");
// predict
// predicted state estimate
math.MatrixMult((float*)F, (float*)X_k_1, n, n, 1, (float*)temp1);
//math.MatrixMult((float*)B, (float*)u, n, p, 1, (float*)X_k_1);
math.MatrixCopy((float*)temp1, n, 1, (float*)X_k_1);
//math.MatrixAdd((float*)temp1, (float*)X_k_1, n, 1, (float*)X_k_1);
//Serial.println("Estimate");
// predicted estimate covariance
math.MatrixMult((float*)F, (float*)P_k_1, n, n, n, (float*)temp2);
math.MatrixMult((float*)temp2, (float*)F_t, n, n, n, (float*)temp3);
math.MatrixAdd((float*)temp3, (float*)Q, n, n, (float*)P_k_1);
//Serial.println("Cov");
// update
// innovation or measurement residual
math.MatrixMult((float*)H, (float*)X_k_1, m, n, 1, (float*)temp4);
math.MatrixSubtract((float*)ProcessedSensorData, (float*)temp4, m, 1, (float*)temp4);
//Serial.println("Res");
// innovation or residual covariance
math.MatrixMult((float*)H, (float*)P_k_1, m, n, n, (float*)tempmn); 

//Serial.println("ResCov");
math.MatrixMult((float*)H_t, (float*)tempnm, m, n, m, (float*)tempm);
math.MatrixAdd((float*)tempm, (float*)R, m, m, (float*)tempm);

// Optimal Kalman gain
math.MatrixInvert((float*)tempm, m);
math.MatrixMult((float*)P_k_1, (float*)tempnm, n, n, m, (float*)tempnm2);
math.MatrixMult((float*)tempnm2, (float*)tempm2, n, m, m, (float*)tempnm);
//Serial.println("KGain");
// updated state estimate
math.MatrixMult((float*)tempnm, (float*)temp2, m, m, 1, (float*)X_k);
math.MatrixAdd((float*)X_k, (float*)X_k_1, m, 1, (float*)X_k);
//Serial.println("State");
//updated estimate covarience
math.MatrixMult((float*)tempnm, (float*)H, n, m, n, (float*)temp1);

int i, j;
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      temp1[i][j] = -temp1[i][j];
    }
  }
  for(i = 0; i < n; i++){
    temp1[i][i] +=1;
  }
  
math.MatrixMult((float*)temp1, (float*)P_k_1, n, n, n, (float*)P_k);
//Serial.println("Ending");
}
