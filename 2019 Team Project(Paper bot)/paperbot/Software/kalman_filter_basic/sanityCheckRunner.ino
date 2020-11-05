//typedef double mtx_type;

mtx_type z[10][1][1] = {
    {{0.39}},
    {{0.50}},
    {{0.48}},
    {{0.29}},
    {{0.25}},
    {{0.32}},
    {{0.34}},
    {{0.48}},
    {{0.41}},
    {{0.45}}
    }; // 10 measurements x (n x 1)
mtx_type u[10][1][1]; // 10 inputs x (p x 1)
int numOfLoops = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Started serial");
  Serial.println("Setting up the u and z matrices");
  //z
  for(int j = 0; j < 10; j++){
    u[j][0][0] = 0;
  }
  Serial.println("Finished setting up the u and z matrices");
  Serial.println("Setting up the filter");
  setupFilter();
  Serial.println("Finished setting up the filter");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(numOfLoops < 10){
    Serial.print("Applying filter for round ");
    Serial.println(numOfLoops+1);
    KalmanFilter filter = applyFilter((mtx_type*)(u[numOfLoops]), (mtx_type*)(z[numOfLoops]));
    Serial.println("xk\tPk");
    Serial.print(kf.x[0][0],4);
    Serial.print("\t");
    Serial.println(kf.P[0][0],4);
    Serial.print("K: ");
    Serial.println(kf.K[0][0],4);
    Serial.print("P-: ");
    Serial.println(kf.test[0][0],4);
    
    numOfLoops++;
  }
}
