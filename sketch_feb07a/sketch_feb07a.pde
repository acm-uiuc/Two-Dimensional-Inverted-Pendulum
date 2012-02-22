#include <MatrixMath.h>
#define n 4
#define p 2
#define m 6 // h m, n;  r 
float F[2][2] = {1,2,1,2};
float F_t[2][2] = {1, 1, 2, 2};
float B[2][2] = {2,1,2,1}; // n*p
float u[2] = {3,1}; // p*1   -> can change when done using
float H[2][2] = {2, 0, 1, 2}; // m*n array
float H_t[2][2]={2, 1, 0, 2}; // transpose of H
float Q[2][2] = {2, 0, 0, 1}; // n*n
float R[2][2] = {1, 0, 0, 1}; // m*m
float Z_k[2] = {2, 1}; // m*1 array -- sensor measurement    -> can change
float P_k_1[2][2] = {1, 3, 0, 2}; // n*n
float X_k_1[2] = {1, 2}; // n*1
float P_k[2][2] = {0, 0, 0, 0}; // n*n
float X_k[2] = {0, 0};  


MatrixMath math;

void KalmanFilter(float* F, // n*n
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
                  float* X_k) // n*1
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
Serial.println("Beginning");
// predict
// predicted state estimate
math.MatrixMult((float*)F, (float*)X_k_1, n, n, 1, (float*)temp1);
math.MatrixMult((float*)B, (float*)u, n, p, 1, (float*)X_k_1);
math.MatrixAdd((float*)temp1, (float*)X_k_1, n, 1, (float*)X_k_1);
Serial.println("Estimate");
// predicted estimate covarience
math.MatrixMult((float*)F, (float*)P_k_1, n, n, n, (float*)temp2);
math.MatrixMult((float*)temp2, (float*)F_t, n, n, n, (float*)temp3);
math.MatrixAdd((float*)temp3, (float*)Q, n, n, (float*)P_k_1);
Serial.println("Cov");
// update
// innovation or measurement residual
math.MatrixMult((float*)H, (float*)X_k_1, m, n, 1, (float*)temp4);
math.MatrixSubtract((float*)Z_k, (float*)temp4, m, 1, (float*)temp4);
Serial.println("Res");
// innovation or residual covarience
math.MatrixMult((float*)H, (float*)P_k_1, m, n, n, (float*)tempmn); 

Serial.println("ResCov");
math.MatrixMult((float*)H_t, (float*)tempnm, m, n, m, (float*)tempm);
math.MatrixAdd((float*)tempm, (float*)R, m, m, (float*)tempm);

// Optimal Kalman gain
math.MatrixInvert((float*)tempm, m);
math.MatrixMult((float*)P_k_1, (float*)tempnm, n, n, m, (float*)tempnm2);
math.MatrixMult((float*)tempnm2, (float*)tempm2, n, m, m, (float*)tempnm);
Serial.println("KGain");
// updated state estimate
math.MatrixMult((float*)tempnm, (float*)temp2, m, m, 1, (float*)X_k);
math.MatrixAdd((float*)X_k, (float*)X_k_1, m, 1, (float*)X_k);
Serial.println("State");
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
Serial.println("Ending");
}

void setup() {
Serial.begin(9600);  

}



void loop ()
{
         Serial.println("\nInitial values of F:"); 
         math.MatrixPrint((float*)F,2,2,"F");
math.MatrixPrint((float*)F_t,2,2,"F_t");
math.MatrixPrint((float*)B,2,2,"B");
math.MatrixPrint((float*)u,1,2,"u");
math.MatrixPrint((float*)H,2,2,"H");
math.MatrixPrint((float*)H_t,2,2,"H_t"); 
math.MatrixPrint((float*)Q,2,2,"Q");
math.MatrixPrint((float*)R,2,2,"R");
math.MatrixPrint((float*)Z_k,1,2,"Z_k");
math.MatrixPrint((float*)P_k_1,2,2,"P_k_1");
math.MatrixPrint((float*)X_k_1,1,2,"X_k_1");
math.MatrixPrint((float*)P_k,2,2,"P_k");
math.MatrixPrint((float*)X_k,1,2,"X_k");

KalmanFilter((float *)F, (float *)F_t, (float *)B, (float *)u, (float *)H, (float *)H_t, (float *)Q, (float *)R, (float *)Z_k, (float *)P_k_1, (float *)X_k_1, (float *)P_k, (float *)X_k);  

         Serial.println("\nend values of F:"); 
         math.MatrixPrint((float*)F,2,2,"F");
math.MatrixPrint((float*)F_t,2,2,"F_t");
math.MatrixPrint((float*)B,2,2,"B");
math.MatrixPrint((float*)u,1,2,"u");
math.MatrixPrint((float*)H,2,2,"H");
math.MatrixPrint((float*)H_t,2,2,"H_t"); 
math.MatrixPrint((float*)Q,2,2,"Q");
math.MatrixPrint((float*)R,2,2,"R");
math.MatrixPrint((float*)Z_k,1,2,"Z_k");
math.MatrixPrint((float*)P_k_1,2,2,"P_k_1");
math.MatrixPrint((float*)X_k_1,1,2,"X_k_1");
math.MatrixPrint((float*)P_k,2,2,"P_k");
math.MatrixPrint((float*)X_k,1,2,"X_k");
Serial.println("Outside");
}

