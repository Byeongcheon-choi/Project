// Note: this uses the MatrixMath library found here: http://playground.arduino.cc/Code/MatrixMath
// The MatrixMath code needs to be added to the Arduino library folder before running
//typedef double mtx_type;
#include <MatrixMath.h>



struct kalman_filter {
  // Set these for the sanity check
  // TODO: Change for actual system
  static const int m = 1; // number of state variables
  static const int n = 1; // number of sensor observations
  static const int p = 1; // number of inputs into system
  
  mtx_type x[m][1]; // m x 1
  mtx_type z[n][1]; // n x 1
  mtx_type u[p][1]; // p x 1
  
  
  mtx_type A[m][m]; // m x m
  mtx_type B[m][p]; // m x p
  mtx_type C[n][m]; // n x m
  mtx_type K[m][n]; // m x n the kalman gain
  mtx_type P[m][m]; // m x m the covariance of the estimation process at step k
  mtx_type Q[m][m]; // m x m covariance of the process noise (e.g. noise from the motors)
  mtx_type R[n][n]; // n x n matrix containing covariance of noise between each of the sensors
  
  mtx_type I[m][m]; // m x m identity matrix
  unsigned long t;
  mtx_type test[1][1];
};

typedef struct kalman_filter KalmanFilter;

KalmanFilter kf;

// sets the initial parameters for the filter and starts keeping time
void setupFilter(){
  //Set up all matrices and constants for filter
  // TODO: Change for actual system
  sanityCheckSetup();
  // Set up I
  for(int j = 0; j < kf.m; j++){
    for(int k = 0; k < kf.m; k++){
      if(j == k){
        kf.I[j][k] = 1;
      } else {
        kf.I[j][k] = 0;
      }
    }
  }
  kf.t = millis();
}

// runs the filter for the elapsed time
KalmanFilter applyFilter(mtx_type* new_u, mtx_type* new_z){
  unsigned long newTime = millis();
  unsigned long deltaT = newTime - kf.t;
  updateModelMatrices(deltaT);

  // set the new u and z matrices
  Matrix.Copy(new_u, kf.p, 1, (mtx_type*) kf.u);
  Matrix.Copy(new_z, kf.n, 1, (mtx_type*) kf.z);
  
  updateFilter();
  kf.t = newTime;
  return kf;
}

// creates new A, B, and C matrices based upon the current estimated state of the model
void updateModelMatrices(unsigned long deltaT){
  // TODO: Change for actual system
    
}

// performs both the time and measurement update of the kalman filter
void updateFilter(){
  // Time Update
  mtx_type x_minus[kf.m][1];
  mtx_type temp1_mx1[kf.m][1];
  mtx_type temp2_mx1[kf.m][1];
  Matrix.Multiply((mtx_type*)kf.A, (mtx_type*)kf.x, kf.m, kf.m, 1, (mtx_type*)temp1_mx1); //Ax
  Matrix.Multiply((mtx_type*)kf.B, (mtx_type*)kf.u, kf.m, kf.p, 1, (mtx_type*)temp2_mx1); //Bu
  Matrix.Add((mtx_type*) temp1_mx1, (mtx_type*) temp2_mx1, kf.m, 1, (mtx_type*) x_minus); // x^- = Ax + Bu

  mtx_type A_transpose[kf.m][kf.m];
  mtx_type P_minus[kf.m][kf.m];
  mtx_type temp3_mxm[kf.m][kf.m];
  mtx_type temp4_mxm[kf.m][kf.m];
  Matrix.Transpose((mtx_type*) kf.A, kf.m, kf.m, (mtx_type*) A_transpose); //A^T
  Matrix.Multiply((mtx_type*)kf.A, (mtx_type*)kf.P, kf.m, kf.m, kf.m, (mtx_type*)temp3_mxm); //AP
  Matrix.Multiply((mtx_type*)temp3_mxm, (mtx_type*)A_transpose, kf.m, kf.m, kf.m, (mtx_type*)temp4_mxm); //APA^T
  Matrix.Add((mtx_type*) temp4_mxm, (mtx_type*) kf.Q, kf.m, kf.m, (mtx_type*) P_minus); //P^- = APA^T + Q
  Matrix.Copy((mtx_type*)P_minus, 1, 1, (mtx_type*) kf.test);

  // Measurement Update
  mtx_type temp5_nxm[kf.n][kf.m];
  mtx_type temp6_nxn[kf.n][kf.n];
  mtx_type temp7_nxn[kf.n][kf.n];
  mtx_type temp8_mxn[kf.m][kf.n];
  mtx_type C_transpose[kf.m][kf.n];
  Matrix.Transpose((mtx_type*) kf.C, kf.n, kf.m, (mtx_type*) C_transpose); //C^T
  Matrix.Multiply((mtx_type*)kf.C, (mtx_type*)P_minus, kf.n, kf.m, kf.m, (mtx_type*)temp5_nxm); //CP
  Matrix.Multiply((mtx_type*)temp5_nxm, (mtx_type*)C_transpose, kf.n, kf.m, kf.n, (mtx_type*)temp6_nxn); //CPC^T
  Matrix.Add((mtx_type*) temp6_nxn, (mtx_type*) kf.R, kf.n, kf.n, (mtx_type*) temp7_nxn); //CPC^T + R
  Matrix.Invert((mtx_type*)temp7_nxn, kf.n);//(CPC^T + R)^-1
  Matrix.Multiply((mtx_type*) C_transpose, (mtx_type*)temp7_nxn, kf.m, kf.n, kf.n, (mtx_type*)temp8_mxn); //C^T(CPC^T + R)^-1
  Matrix.Multiply((mtx_type*)P_minus, (mtx_type*)temp8_mxn, kf.m, kf.m, kf.n, (mtx_type*)kf.K); //K = C^T(CPC^T + R)^-1

  mtx_type temp9_nx1[kf.n][1];
  mtx_type temp10_nx1[kf.n][1];
  Matrix.Multiply((mtx_type*)kf.C, (mtx_type*)kf.x, kf.n, kf.m, 1, (mtx_type*) temp9_nx1); //Cx
  Matrix.Subtract((mtx_type*) kf.z, (mtx_type*) temp9_nx1, kf.n, 1, (mtx_type*) temp10_nx1); //z-Cx
  Matrix.Multiply((mtx_type*)kf.K, (mtx_type*)temp10_nx1, kf.m, kf.n, 1, (mtx_type*)temp1_mx1); //K(z-Cx)
  Matrix.Add((mtx_type*)kf.x, (mtx_type*) temp1_mx1, kf.m, 1, (mtx_type*) temp2_mx1); //x + K(z-Cx)
  Matrix.Copy((mtx_type*)temp2_mx1, kf.m, 1, (mtx_type*) kf.x); //x = x + K(z-Cx)
  

  Matrix.Multiply((mtx_type*)kf.K, (mtx_type*)kf.C, kf.m, kf.n, kf.m, (mtx_type*)temp3_mxm); //KC
  Matrix.Subtract((mtx_type*) kf.I, (mtx_type*) temp3_mxm, kf.m, kf.m, (mtx_type*) temp4_mxm); //I-KC
  Matrix.Multiply((mtx_type*)temp4_mxm, (mtx_type*)P_minus, kf.m, kf.m, kf.m, (mtx_type*)kf.P); //P=(I-KC)P^-
}

// set up and run a kalman filter for the following scenario: http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
// this will provide a basic check that things are running as expected
// note that there's only one sensor, no control signal, and the model is independent of time. p = 1 so that way the declarations still compile
void sanityCheckSetup(){
  
  kf.x[0][0] = 0;
  kf.z[0][0] = 0;
  kf.u[0][0] = 0;
  

  kf.A[0][0] = 1;
  kf.B[0][0] = 0;
  kf.C[0][0] = 1;
  kf.K[0][0] = 0;
  kf.P[0][0] = 1;
  kf.Q[0][0] = 0;
  kf.R[0][0] = 0.1; //standard deviation is 0.1, so variance is (0.1)^2
}
