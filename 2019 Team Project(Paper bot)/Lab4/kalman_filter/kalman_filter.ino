// Note: this uses the MatrixMath library found here: http://playground.arduino.cc/Code/MatrixMath
// The MatrixMath code needs to be added to the Arduino library folder before running
//typedef double mtx_type;
#include <MatrixMath.h>
#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"

#include <Wire.h>
#include <VL53L0X.h>

const double start_theta = (3*PI/2);
const double start_x = 300; // max of 500
const double start_y = 750-200; // max of 750



struct kalman_filter {
  // TODO: Change for actual system
  static const int m = 3; // number of state variables
  static const int n = 3; // number of sensor observations
  static const int p = 2; // number of inputs into system
  
  mtx_type x[m][1]; // m x 1
  mtx_type z[n][1]; // n x 1s
  mtx_type u[p][1]; // p x 1
  
  
  mtx_type A[m][m]; // m x m
  mtx_type B[m][p]; // m x p
  mtx_type C[n][m]; // n x m (a.k.a. H matrix), which is the Jacobian of the h(state) equations
  mtx_type h[n][1]; // n x 1 that contatins the value of the h(state) function *NOTE*: h_function thinks this is a [1][3]...
  mtx_type K[m][n]; // m x n the kalman gain
  mtx_type P[m][m]; // m x m the covariance of the estimation process at step k
  mtx_type Q[m][m]; // m x m covariance of the process noise (e.g. noise from the motors)
  mtx_type R[n][n]; // n x n matrix containing covariance of noise between each of the sensors
  mtx_type P_minus[m][m];
  mtx_type x_minus[m][1];
  mtx_type Bu[m][1]; // m x 1 matrix of B times u
  
  mtx_type I[m][m]; // m x m identity matrix
  mtx_type test1[n][1];
  mtx_type test2[m][1];
  mtx_type test3[m][1];
  mtx_type test4[n][n];
  mtx_type test5[n][n];
  mtx_type test6[m][n];
  
  unsigned long t;
  boolean isSetup = false;
  unsigned long iterationNumber;
};

typedef struct kalman_filter KalmanFilter;

KalmanFilter kf;


// sets the initial parameters for the filter and starts keeping time
KalmanFilter setupFilter(){
  //Set up all matrices and constants for filter

  kf.x[0][0] = start_theta;
  kf.x[1][0] = start_x;
  kf.x[2][0] = start_y;

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

  // Set up A (just partially, rest done in update filter)
  Matrix.Copy((mtx_type*)kf.I, kf.m, kf.m, (mtx_type*) kf.A);

  // Set up B
  for(int j = 0; j < kf.m; j++){
    for(int k = 0; k < kf.p; k++){
      kf.B[j][k] = 0;
    }
  }

  // Set up C
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 3; k++){
      if(j == 0 && k == 0){
        kf.C[j][k] = 1;
      } else if(j == 1 && k == 2){
        kf.C[j][k] = -8.20;
      } else if(j == 2 && k == 1){
        kf.C[j][k] = -8.59;
      } else {
        kf.C[j][k] = 0;
      }
    }
   }

  // Set up P
  for(int j = 0; j < kf.m; j++){
    for(int k = 0; k < kf.m; k++){
      if(j==k){
        kf.P[j][k] = 0.5;
      }
    }
  }
  //Matrix.Copy((mtx_type*)kf.I, kf.m, kf.m, (mtx_type*) kf.P);

   // Set up Q
  for(int j = 0; j < kf.m; j++){
    for(int k = 0; k < kf.m; k++){
      if(j==0 && k==0){
        kf.Q[j][k] = 0.002;
      } else if(j==k){
        kf.Q[j][k] = 0.002;
      }
    }
  }

  // Set up R
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 3; k++){
      if(j == 0 && k == 0){
        //kf.R[j][k] = 2.95 * (PI/180);
        kf.R[j][k] = 1;
      } else if(j == 1 && k == 1){
        kf.R[j][k] = 2.161;
      } else if(j == 2 && k == 2){
        kf.R[j][k] = 2.844;
      } else {
        kf.R[j][k] = 0;
      }
    }
   }

  
  kf.t = millis();
  kf.isSetup = true;
  kf.iterationNumber = 0;
  return kf;
}

// runs the filter for the elapsed time
KalmanFilter applyFilter(mtx_type* new_u, mtx_type* new_z){
  unsigned long newTime = millis();
  unsigned long deltaT = (newTime - kf.t); //milliseconds
  updateModelMatrices(deltaT, new_u);

  // set the new u and z matrices
  Matrix.Copy(new_u, kf.p, 1, (mtx_type*) kf.u);
  Matrix.Copy(new_z, kf.n, 1, (mtx_type*) kf.z);
  
  updateFilter();
  kf.t = newTime;
  kf.iterationNumber = kf.iterationNumber + 1;
  return kf;
}

// creates new A, B, and C matrices based upon the current estimated state of the model (deltaT in milliseconds)
void updateModelMatrices(unsigned long deltaT, mtx_type* new_u){
   // TODO: Change for actual system
   Serial.print("deltaT: ");
   Serial.println(deltaT);
  double vl;
  double vr;
   if(new_u[0] < 105 && new_u[0] > 85){
      vl=0;
   } else{
       vl = (0.09 * tanh((new_u[0]/15)-6));
   }
   
   if(new_u[1] < 102 && new_u[1] > 70){
      vr=0;
   } else{
       vr = 0.095 * tanh((new_u[1]/25)-3.6);
   }

//   Serial.print("vl: ");
//   Serial.println(vl, 8);
//   Serial.print("vr: ");
//   Serial.println(vr, 8);
   double average_v = (vl+vr)/2;
   kf.Bu[0][0] = deltaT * ((vr-vl)/84);
   kf.Bu[1][0] = deltaT * average_v * sin(kf.x[0][0]);
   kf.Bu[2][0] = deltaT * average_v * cos(kf.x[0][0]);
//   Serial.print("average_v: ");
//   Serial.println(average_v, 8);
   
//   kf.A[1][0] = average_v * -1 * sin(kf.x[0][0]) * deltaT;
//   kf.A[2][0] = average_v * cos(kf.x[0][0]) * deltaT;

//   double temp1 = (4340*pow(1.14,new_u[0]))/pow((pow(1.14,new_u[0])) + 162755,2);
//   Serial.print("temp1: ");
//   Serial.println(temp1, 8);
//   double temp2 = (22.5*pow(1.08,new_u[1]))/pow((pow(1.083,new_u[1])) + 1339,2);
//   Serial.print("temp2: ");
//   Serial.println(temp2, 8);
//   kf.B[0][0] = (1/84)*temp1*deltaT;
//   kf.B[0][1] = (-1/84)*temp2*deltaT;
//   kf.B[1][0] = cos(kf.x[0][0]/2)*temp1*deltaT;
//   kf.B[1][1] = cos(kf.x[0][0]/2)*temp2*deltaT;
//   kf.B[2][0] = sin(kf.x[0][0]/2)*temp1*deltaT;
//   kf.B[2][1] = sin(kf.x[0][0]/2)*temp2*deltaT;


}

// performs both the time and measurement update of the kalman filter
void updateFilter(){

//*************************Think that the A matrix needs to be an actual Jacobian, not just identity matrix)***********************
  
  // Time Update
  mtx_type temp1_mx1[kf.m][1];
  mtx_type temp2_mx1[kf.m][1];
  Matrix.Multiply((mtx_type*)kf.A, (mtx_type*)kf.x, kf.m, kf.m, 1, (mtx_type*)temp1_mx1); //Ax
  //Matrix.Multiply((mtx_type*)kf.B, (mtx_type*)kf.u, kf.m, kf.p, 1, (mtx_type*)temp2_mx1); //Bu
  Matrix.Add((mtx_type*) temp1_mx1, (mtx_type*) kf.Bu, kf.m, 1, (mtx_type*) kf.x_minus); // x^- = Ax + Bu

  mtx_type A_transpose[kf.m][kf.m];
  mtx_type temp3_mxm[kf.m][kf.m];
  mtx_type temp4_mxm[kf.m][kf.m];
  Matrix.Transpose((mtx_type*) kf.A, kf.m, kf.m, (mtx_type*) A_transpose); //A^T
  //Matrix.Print((mtx_type*)A_transpose, kf.m, kf.m, "A^T");
  Matrix.Multiply((mtx_type*)kf.A, (mtx_type*)kf.P, kf.m, kf.m, kf.m, (mtx_type*)temp3_mxm); //AP
  Matrix.Multiply((mtx_type*)temp3_mxm, (mtx_type*)A_transpose, kf.m, kf.m, kf.m, (mtx_type*)temp4_mxm); //APA^T
  Matrix.Add((mtx_type*) temp4_mxm, (mtx_type*) kf.Q, kf.m, kf.m, (mtx_type*) kf.P_minus); //P^- = APA^T + Q

  //update C matrix before
  //-------------- Update C/H matrix ------------------------
  h_function(true,kf.C[0],1,kf.x_minus[0][0],kf.x_minus[1][0],kf.x_minus[2][0]);//update first row (function 1, measured theta) TODO
  h_function(true,kf.C[1],2,kf.x_minus[0][0],kf.x_minus[1][0],kf.x_minus[2][0]);//update second row (function 2, front distance) TODO
  h_function(true,kf.C[2],3,kf.x_minus[0][0],kf.x_minus[1][0],kf.x_minus[2][0]);//update third row (function 3, side distance) TODO
//  unsigned int ptr1 = (unsigned int) &kf.C[0][0];
//  unsigned int ptr2 = (unsigned int) &kf.C[1][0];
//  unsigned int ptr3 = (unsigned int ) &(kf.C[2][0]);
//  Serial.print("C[0]: ");
//  Serial.println(ptr1);
//  Serial.print("C[1]: ");
//  Serial.println((ptr2));
//  Serial.print("C[2]: ");
//  Serial.println((ptr3));

  // Measurement Update
  mtx_type temp5_nxm[kf.n][kf.m];
  mtx_type temp6_nxn[kf.n][kf.n];
  mtx_type temp7_nxn[kf.n][kf.n];
  mtx_type temp8_mxn[kf.m][kf.n];
  mtx_type C_transpose[kf.m][kf.n];
  Matrix.Transpose((mtx_type*) kf.C, kf.n, kf.m, (mtx_type*) C_transpose); //C^T
  Matrix.Multiply((mtx_type*)kf.C, (mtx_type*)kf.P_minus, kf.n, kf.m, kf.m, (mtx_type*)temp5_nxm); //CP
  Matrix.Multiply((mtx_type*)temp5_nxm, (mtx_type*)C_transpose, kf.n, kf.m, kf.n, (mtx_type*)temp6_nxn); //CPC^T
  Matrix.Copy((mtx_type*)temp6_nxn, kf.n, kf.n, (mtx_type*) kf.test4);
  Matrix.Add((mtx_type*) temp6_nxn, (mtx_type*) kf.R, kf.n, kf.n, (mtx_type*) temp7_nxn); //CPC^T + R
  int inversion = Matrix.Invert((mtx_type*)temp7_nxn, kf.n);//(CPC^T + R)^-1
  Matrix.Copy((mtx_type*)temp7_nxn, kf.n, kf.n, (mtx_type*) kf.test5);
  if(inversion == 0){
    DEBUG("*********************************************************************************************************");
    DEBUG("WE FOUND A NON-INVERTIBLE MATRIX when calculating K!!!");
    for(int j = 0; j < 10; j++){
      DEBUG("*********************************************************************************************************");
    }
  }
  Matrix.Multiply((mtx_type*) C_transpose, (mtx_type*)temp7_nxn, kf.m, kf.n, kf.n, (mtx_type*)temp8_mxn); //C^T(CPC^T + R)^-1
  Matrix.Copy((mtx_type*)temp8_mxn, kf.m, kf.n, (mtx_type*) kf.test6);
  Matrix.Multiply((mtx_type*)kf.P_minus, (mtx_type*)temp8_mxn, kf.m, kf.m, kf.n, (mtx_type*)kf.K); //K = (P-)C^T(CPC^T + R)^-1
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      if(j != k) {
        kf.K[j][k] = 0;
      }
    }
  }

//  mtx_type temp9_nx1[kf.n][1];
  mtx_type temp10_nx1[kf.n][1];
//  Matrix.Multiply((mtx_type*)kf.C, (mtx_type*)kf.x, kf.n, kf.m, 1, (mtx_type*) temp9_nx1); //Cx
//  Matrix.Copy((mtx_type*)temp9_nx1, kf.n, 1, (mtx_type*) kf.test1);
  

  //h(x)
  h_function(false,kf.h[0],0,kf.x_minus[0][0],kf.x_minus[1][0],kf.x_minus[2][0]);
  
  //z-h(x)
  Matrix.Subtract((mtx_type*) kf.z, (mtx_type*) kf.h, kf.n, 1, (mtx_type*) temp10_nx1); //z-h(x)
  
  Matrix.Copy((mtx_type*)temp10_nx1, kf.n, 1, (mtx_type*) kf.test2);
  Matrix.Multiply((mtx_type*)kf.K, (mtx_type*)temp10_nx1, kf.m, kf.n, 1, (mtx_type*)temp1_mx1); //K(z-Cx)
  Matrix.Copy((mtx_type*)temp1_mx1, kf.m, 1, (mtx_type*) kf.test3);
  Matrix.Add((mtx_type*)kf.x_minus, (mtx_type*) temp1_mx1, kf.m, 1, (mtx_type*) temp2_mx1); //x- + K(z-Cx)
  Matrix.Copy((mtx_type*)temp2_mx1, kf.m, 1, (mtx_type*) kf.x); //x = x + K(z-Cx)
  

  Matrix.Multiply((mtx_type*)kf.K, (mtx_type*)kf.C, kf.m, kf.n, kf.m, (mtx_type*)temp3_mxm); //KC
  Matrix.Subtract((mtx_type*) kf.I, (mtx_type*) temp3_mxm, kf.m, kf.m, (mtx_type*) temp4_mxm); //I-KC
  Matrix.Multiply((mtx_type*)temp4_mxm, (mtx_type*)kf.P_minus, kf.m, kf.m, kf.m, (mtx_type*)kf.P); //P=(I-KC)P^-

  //Matrix.Copy((mtx_type*)kf.x_minus, kf.m, 1, (mtx_type*) kf.x); //Put this line in if you want to disable the KF 
}

//// set up and run a kalman filter for the following scenario: http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
//// this will provide a basic check that things are running as expected
//// note that there's only one sensor, no control signal, and the model is independent of time. p = 1 so that way the declarations still compile
//void sanityCheckSetup(){
//  
//  kf.x[0][0] = 0;
//  kf.z[0][0] = 0;
//  kf.u[0][0] = 0;
//
//  kf.A[0][0] = 1;
//  kf.B[0][0] = 0;
//  kf.C[0][0] = 1;
//  kf.K[0][0] = 0;
//  kf.P[0][0] = 1;
//  kf.Q[0][0] = 0;
//  kf.R[0][0] = 0.1; //standard deviation is 0.1, so variance is (0.1)^2
//}

//double tanh(double val){
//  double temp1 = exp(val);
//  double temp2 = exp(-1*val);
//  return ((temp1 - temp2)/(temp1+temp2));
//}

//calculate the h function for the front and side sensors
//the general strategy is calculate all 4 equations and then select the one that is smallest positive value
//if calcJacobian, this will return the Jacobian value and NOT the minimum, positive distance!
// h is a pointer to the array to be updated (could be h (1xn) or a temp value (1xn) to be used in updating C)
// functionNum specifies, when calculating the Jacobian, which sensor values the function should return e.g. 1 -> f1 = theta measured, 2 -> f2 = dfront, 3 -> f3 = dside
void h_function(bool calcJacobian, mtx_type* h, int functionNum, double theta, double x, double y) {
    const double xd = 500; //width is 500 mm
    const double yd = 750; //length/height is 750 mm
    double topFront;
    double rightFront;
    double leftFront;
    double botFront;
    double topSide;
    double rightSide;
    double leftSide;
    double botSide;

    double thetaResult = theta + (1.05*(PI/180));
//    Serial.print("h: ");
//    Serial.println((unsigned int) &h[0]);

      //top wall equation, front sensor
      topFront = (yd - y) / cos(theta);

      //right wall equation, front sensor
      rightFront = (xd - x) / sin(theta);

      //bottom wall equation, front sensor
      botFront = y / (-1 * cos(theta));

      //left wall equation, front sensor
      leftFront = x / (-1 * sin(theta));
    
      //must be the side sensor
      //top wall equation, side sensor
      topSide = (yd - y) / (-1*sin(theta));

      //right wall equation, side sensor
      rightSide = (xd - x) / cos(theta);

      //bottom wall equation, side sensor
      botSide = y / (sin(theta));

      //left wall equation, side sensor
      leftSide = x / (-1 * cos(theta));
    

  double valuesFront[4] = {topFront, botFront, leftFront, rightFront};
  double minPositiveFront = -1;
  int wallIndexFront = -1; //stores the index of the wall that the sensor is reading off of
  for (int i = 0; i < 4; i++) {
    if (valuesFront[i] > 0 && minPositiveFront < 0) {//this means minPositive hasn't been initialized yet
      minPositiveFront = valuesFront[i];
      wallIndexFront = i;
    }
    else if (valuesFront[i] > 0 && minPositiveFront > 0 && valuesFront[i] < minPositiveFront) {
      minPositiveFront = valuesFront[i];
      wallIndexFront = i;
    }  
  }

  double valuesSide[4] = {topSide, botSide, leftSide, rightSide};
  double minPositiveSide = -1;
  int wallIndexSide = -1; //stores the index of the wall that the sensor is reading off of
  for (int i = 0; i < 4; i++) {
    if (valuesSide[i] > 0 && minPositiveSide < 0) {//this means minPositive hasn't been initialized yet
      minPositiveSide = valuesSide[i];
      wallIndexSide = i;
    }
    else if (valuesSide[i] > 0 && minPositiveSide > 0 && valuesSide[i] < minPositiveSide) {
      minPositiveSide = valuesSide[i];
      wallIndexSide = i;
    }  
  }

  if (!calcJacobian) {
    h[0] = thetaResult;
    h[1] = minPositiveFront - 29;
    h[2] = minPositiveSide - 19; //Originally was 29
//    Serial.print("Wall Index Front: ");
//    Serial.println(wallIndexFront);
//    Serial.print("Wall Index Side: ");
//    Serial.println(wallIndexSide);
    return;
  }
  else {
//        Serial.print("Wall Index Front: ");
//    Serial.println(wallIndexFront);
//    Serial.print("Wall Index Side: ");
//    Serial.println(wallIndexSide);
    if (functionNum == 1) {//return Jacobian for function 1 (theta measured)
      h[0] = 1;
      h[1] = 0;
      h[2] = 0;
      return;
      //This one is Andrew's and Calvin's personal favorite! (ENJOY!!!)
    }
    //depending on wallIndex (the wall the sensor is measuring off of), we need to use a certain Jacobian equation
    else if (functionNum == 2) { //return Jacobian for function 2 (front sensor distance)
      if (wallIndexFront == 0) {//top wall equation, front sensor
        h[0] = (sin(theta)*(yd - y)/(pow(cos(theta),2)))/1000;
        h[1] = 0;
        h[2] = -1/(cos(theta));
        return;
      }
      else if (wallIndexFront == 1){//bottom wall equation, front sensor
        h[0] = (-1*sin(theta)*(y)/(pow(cos(theta),2)))/1000;
        h[1] = 0;
        h[2] = -1/(cos(theta));
        return;
      }
      else if (wallIndexFront == 2){//left wall equation, front sensor
//        Serial.println("Front Sensor, Left wall equation");

        h[0] = ((cos(theta) * x)/(pow(sin(theta),2)))/1000;
        h[1] = -1/sin(theta);
        h[2] = 0;
//        Serial.print("h[0]: ");
//        Serial.println(h[0]);
//        Serial.print("h[1]: ");
//        Serial.println(h[1]);
//        Serial.print("h[2]: ");
//        Serial.println(h[2]);
        return;
      }
      else if (wallIndexFront == 3){//right wall equation, front sensor
        h[0] = ((cos(theta) * (x-xd))/(pow(sin(theta),2)))/1000;
        h[1] = -1/sin(theta);
        h[2] = 0;
        return;
      }
      //should NEVER get here!
      return;
    }
    else if (functionNum == 3) {//return Jacobian for function 3 (side sensor distance)
      if (wallIndexSide == 0) {//top wall equation, side sensor
        h[0] = (-1*(cos(theta)*(y-yd))/(pow(sin(theta),2)))/1000;
        h[1] = 0;
        h[2] = 1/sin(theta);
        return;
      }
      else if (wallIndexSide == 1){//bottom wall equation, side sensor
        h[0] = (-1*(cos(theta)*(y))/(pow(sin(theta),2)))/1000;
        h[1] = 0;
        h[2] = 1/sin(theta);
        return;
      }
      else if (wallIndexSide == 2){//left wall equation, side sensor
        h[0] = (-1*(sin(theta)*x)/(pow(cos(theta),2)))/1000;
        h[1] = -1/(cos(theta));
        h[2] = 0;
        return;
      }
      else if (wallIndexSide == 3){//right wall equation, side sensor
        h[0] = ((sin(theta)*(xd -x))/(pow(cos(theta),2)))/1000;
        h[1] = -1/(cos(theta));
        h[2] = 0;
        return;
      }
      //should NEVER get here!
      return;
    }
  }
}
