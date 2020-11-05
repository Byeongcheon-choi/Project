

#include <Arduino.h>
#include <MatrixMath.h>



VL53L0X sensor; //yellow wire is lidar 1 (D3)
VL53L0X sensor2;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT D5
#define SCL_PORT D6

char tx_buffer[50];

float biases[3] = {-66, 53, -30};
//float biases[3] = {0, 0, 0};
float scales[3] = {0.978, 1.026, 0.997};
double firstAngle = -1;

mtx_type u[2][1];
mtx_type z[3][1];

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 87; // 87 are the defaults for our robot
int servo_right_ctr = 87;


// WiFi AP parameters
char ap_ssid[21];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

uint8_t clientId =-1;

boolean jayhawksWebSocketIsConnected = false;

String html;
String css;

boolean has_commands = false;  // set as true until the client connects since it has nothing to do
boolean pending = false;
float commands[6] = {0, 0, 0, 0, 0, 0}; // two instructions, (leftPWM, rightPWM, timeToExecute) then the next command
int instruction_num = 0;
int start_time = -1;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setup() {

//    ESP.wdtDisable();
//    ESP.wdtEnable(WDTO_8S);
  
    setupPins();

    sprintf(ap_ssid, "Jayhawk_ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();
    DEBUG("Setting up the sensors");
    // Setting up the sensors:
    Wire.begin(SDA_PORT,SCL_PORT);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    // Setting up lidar sensor 1
    digitalWrite(D3, HIGH);
    delay(150);
    sensor.init(true);
    delay(100);
    sensor.setAddress((uint8_t)22);
    // Setting up lidar sensor 1
    digitalWrite(D4, HIGH);
    delay(150);
    sensor2.init(true);
    delay(100);
    sensor2.setAddress((uint8_t)25);
    DEBUG("Setting up the filter");
    setupFilter();

    DEBUG("Out of the setup");
}

void loop() {
    ESP.wdtFeed();
    //DEBUG("Trying to divide by zero");
//    int var = 10 / 0;
    //Serial.println(var);
    //DEBUG("We could divide by zero");
    wsLoop();
    //httpLoop();
    //DEBUG("Got through the httpLoop");
    // TODO: Get the sensor data and input into the filter
    double s1Reading = sensor.readRangeSingleMillimeters();
    //DEBUG("Got through the first lidar reading");
    //ESP.wdtFeed();
    double s2Reading = sensor2.readRangeSingleMillimeters();
    //DEBUG("Got through the two lidar readings");
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    // Read register Status 1 and wait for the DRDY: Data Ready
    uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));
    // Read magnetometer data  
    uint8_t Mag[7];  
    I2Cread(MAG_ADDRESS,0x03,7,Mag);
    // Magnetometer
    int16_t mx=(Mag[1]<<8 | Mag[0]);
    int16_t my=(Mag[3]<<8 | Mag[2]);
    int16_t mz=(Mag[5]<<8 | Mag[4]);
  
    mx = (mx - biases[0])  * scales[0];
    my = (my - biases[1]) * scales[1];
    mz = (mz - biases[2]) * scales[2];
    float heading = atan2(mx, my);
  
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.1920;
    heading += declinationAngle;
    // Subtract by the first angle, that way whenever the paperbot starts up its initial direction is considered 0
    if(firstAngle == -1){
      firstAngle = heading;
      Serial.print("First Angle: ");
      Serial.println(firstAngle);
    }
    heading = heading - firstAngle;
    heading = heading + start_theta;
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
    // Make sure that the angle is within PI of the current state, that way we add/subtract the corrections properly
    while(abs(heading - kf.x[0][0]) > PI){
      if(heading - kf.x[0][0] > PI){
        heading = heading - (2*PI);
      } else if(kf.x[0][0] - heading > PI) {
        heading = heading + (2*PI);
      }
    }
    
    ESP.wdtFeed();
    
    //Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI; 
      // input data into the filter
      z[0][0] = heading;
      z[1][0] = (((s1Reading-60.4)/8.6)*10);
      z[2][0] = (((s2Reading-59.7)/8.2)*10);
      //drive(180-180, 121);
      u[0][0] = 180-servo_left.read();
      u[1][0] = servo_right.read();
      kf = applyFilter((mtx_type*)u,(mtx_type*)z);
//      printAll(kf);
     Matrix.Print((mtx_type*)kf.x, kf.m, 1, "xk");
     Matrix.Print((mtx_type*)kf.x_minus, kf.m, 1, "x-");
     Matrix.Print((mtx_type*)kf.z, kf.n, 1, "z");
    
//     Matrix.Print((mtx_type*)kf.K, kf.m, kf.n, "K");
      //delay(500);
      if(!has_commands && !pending && jayhawksWebSocketIsConnected) {
        pending = true;
        ESP.wdtFeed();
        transmitState(kf);
//        inRange = false;
      }
      if(has_commands) {
        if(start_time == -1) {  // just got the commands
          instruction_num = 0;
          start_time = millis();
        }
        int time_elapsed = millis() - start_time;
        if(instruction_num == 0 && time_elapsed >= commands[2]) { // the time for the first command
          instruction_num = 1;
          start_time = millis();
          time_elapsed = 0;
        } else if(instruction_num == 1 && time_elapsed >= commands[5]) { // the time for the first command
          instruction_num = 2;
          has_commands = false;
        }
        if(has_commands) { // check this again as we could possibly be done
          drive(180 - commands[0 + (instruction_num*3)], commands[1 + (instruction_num*3)]);
        } else {
          stop();
        }
        
      } else {
        stop();
      }
      yield();
    
    //DEBUG("FINISHED LOOP");
}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void right() {
  DEBUG("right");
  drive(180, 180);
}

void left() {
  DEBUG("left");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");

    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    digitalWrite(D7, LOW);
    digitalWrite(D8, LOW);
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            jayhawksWebSocketIsConnected = false;
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            clientId = id;
            jayhawksWebSocketIsConnected = true;
            has_commands = false;
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if(payload[0] == '&') {
              has_commands = true;
              pending = false;
              start_time = -1;
              // for each incoming line
              int character_num = 2;
              String temp;
              for(int k = 0; k < 6; k++) {
                temp = "";
                while(payload[character_num] != '\n') {
                  temp += (char)payload[character_num];
                  character_num++;
                }
                commands[k] = temp.toFloat();
                character_num++; // get past the new line character
                Serial.print("k: ");
                Serial.println(k);
                Serial.println(commands[k]);
              }
              
            }
            
            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  //wsSend(id, "Hello world!");
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
//                  Serial.println("Zero @ (" + servo_left_ctr + ", " + servo_right_ctr + ")");
                  Serial.println(tx);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}

void printAll(KalmanFilter kf){
  
  Serial.println("------------------------------------------------");
  Serial.println(kf.iterationNumber);
    //Serial.println("xk\tPk");
    Matrix.Print((mtx_type*)kf.x, kf.m, 1, "xk");
    Matrix.Print((mtx_type*)kf.z, kf.n, 1, "z");
    Matrix.Print((mtx_type*)kf.u, kf.p, 1, "u");
    Matrix.Print((mtx_type*)kf.P, kf.m, kf.m, "Pk");
    Matrix.Print((mtx_type*)kf.K, kf.m, kf.n, "K");
    Matrix.Print((mtx_type*)kf.P_minus, kf.m, kf.m, "P-");
//    Matrix.Print((mtx_type*)kf.A, kf.m, kf.m, "A");
//    Matrix.Print((mtx_type*)kf.B, kf.m, kf.p, "B");
    Matrix.Print((mtx_type*)kf.C, kf.n, kf.m, "C");
    Matrix.Print((mtx_type*)kf.h, kf.n, 1, "h");
    Matrix.Print((mtx_type*)kf.x_minus, kf.m, 1, "x-");
    Matrix.Print((mtx_type*)kf.Bu, kf.m, 1, "Bu");
//    Matrix.Print((mtx_type*)kf.Q, kf.m, kf.m, "Q");
//    Matrix.Print((mtx_type*)kf.R, kf.n, kf.n, "R");
//    Matrix.Print((mtx_type*)kf.test1, kf.n, 1, "test1");
    Matrix.Print((mtx_type*)kf.test2, kf.m, 1, "test2");
    Matrix.Print((mtx_type*)kf.test3, kf.m, 1, "test3");
    Matrix.Print((mtx_type*)kf.test4, kf.n, kf.n, "test4");
    Matrix.Print((mtx_type*)kf.test5, kf.n, kf.n, "test5");
    Matrix.Print((mtx_type*)kf.test6, kf.m, kf.n, "test6");
    
//    Serial.print("Angle: ");
//    Serial.println(kf.x[0][0] * 180/PI,4);
//    Serial.print("X: ");
//    Serial.println(kf.x[1][0],4);
//    Serial.print("Y: ");
//    Serial.println(kf.x[2][0],4);
}

void transmitState(KalmanFilter kf){
  Serial.println("Transmiting the state to the browser");
    //Serial.println("xk\tPk");
    String toSend = String("%");
//    toSend.concat("------------------------------------\n");
    toSend.concat("\n");
//    toSend.concat(kf.iterationNumber);
    toSend.concat(matrixToString((mtx_type*)kf.x, kf.m, 1, "xk"));
//    toSend.concat(matrixToString((mtx_type*)kf.z, kf.n, 1, "z"));
    
//    Serial.print("Angle: ");
//    Serial.println(kf.x[0][0] * 180/PI,4);
//    Serial.print("X: ");
//    Serial.println(kf.x[1][0],4);
//    Serial.print("Y: ");
//    Serial.println(kf.x[2][0],4);


    memset(tx_buffer, 0, sizeof(tx_buffer));
    Serial.print("Size of toSend: ");
    Serial.println(toSend.length());
    toSend.toCharArray(tx_buffer,sizeof(tx_buffer));
    wsSend(clientId, tx_buffer);
}

// modified function from MatrixMath library so we can print a matrix to a string
String matrixToString(mtx_type* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  String output = String("\n" + label + "\n");
  int i, j;
//  Serial.println();
//  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      String temp = String(A[n * i + j],5);
      temp.concat("\t");
      output.concat(temp);
    }
    output.concat("\n");
//    Serial.println();
  }
  return output;
}
