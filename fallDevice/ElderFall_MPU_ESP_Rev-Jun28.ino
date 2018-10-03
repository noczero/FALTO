#include <I2Cdev.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU6050_6Axis_MotionApps20_10hz_esp32.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_PIN 2             //Onboard Active-low LED
#define PWR_PIN 16            //Pin connected to regulator's EN
#define MFB_INTPIN 13         

MPU6050 mpu;
#define MPU_INTPIN 12         //Pin for interrupt from MPU
#define MPU_ADDR 0x68

//const char* ssid = "M0Z3";
//const char* password = "rewqaz13";
//
//#define mqtt_server "192.168.43.25"
//#define mqtt_port 1883

const char* ssid = "Mydlink-1";
const char* password = "satria1234";

#define mqtt_server "192.168.43.106"
#define mqtt_port 1883


#define device_name "DEV001"
#define mqtt_topic "DP001/"

WiFiClient espClient;
PubSubClient client(espClient);

#define samPacketLength 50
#define samMaxLength (30*samPacketLength)
unsigned int samCount=0;
unsigned int samLength=0;
char samChar[samMaxLength];

// ================================================================
// ===                     MPU VARIABLES                        ===
// ================================================================

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===                     MPU INTERRUPT                        ===
// ================================================================

void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      SETUP WIFI                          ===
// ================================================================

void WIFI_SET() {  
    delay(10);  
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        LED_BLINK(50,200,2);
        Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("WiFi connected");
    client.setServer(mqtt_server, mqtt_port);
    LED(500);
}


// ================================================================
// ===                   RECONNECT (MQTT)                       ===
// ================================================================

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(device_name)) {
    Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

// ================================================================
// ===                    POWER-RELATED                         ===
// ================================================================

float batteryVoltage(){
  int rawADC=analogRead(A0);
  float voltage=float(rawADC)/1023.00*5.52;
  return voltage;
}

void PWR_INIT(){
  digitalWrite(PWR_PIN,HIGH);
  pinMode(16,OUTPUT);
}

void multiBtn(){
  LED(500);
  if(digitalRead(MFB_INTPIN)){
    delay(500);
    if(digitalRead(MFB_INTPIN)){
      LED_BLINK(50,200,3);
      digitalWrite(16,LOW);
    }
  }  
}

// ================================================================
// ===                    LED INDICATOR                         ===
// ================================================================

void LED_INIT(){
  digitalWrite(LED_PIN,HIGH);
  pinMode(LED_PIN,OUTPUT);
}

void LED(int timeOn){
  digitalWrite(LED_PIN,LOW);
  delay(timeOn);
  digitalWrite(LED_PIN,HIGH);
}

void LED_ON(){
  digitalWrite(LED_PIN,LOW);
}
void LED_OFF(){
  digitalWrite(LED_PIN,HIGH);
}

void LED_BLINK(int onTime,int offTime, int blinkCount){
  while(blinkCount>0){
  LED(onTime);
  delay(offTime);
  blinkCount--;
  }
}

// ================================================================
// ===                   SAMPLE TO CHAR ARRAY                   ===
// ================================================================

void addSam(char head,int buff){
  samChar[samLength]=head;  samLength++;
  if(buff < 0){samChar[samLength]=45;  buff *= -1; samLength++;}
  if(buff > 9){samChar[samLength]=(48+(buff/10));  samLength++;}
  samChar[samLength]=(48+(buff%10)); samLength++;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(MPU_INTPIN,INPUT);
  delay(200);                   //push time before device really turned on
  PWR_INIT();
  LED_INIT();
  LED(500);
  Serial.begin(115200);
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage());
  
  WIFI_SET(); 
  
  Wire.begin();
  Wire.setClock(400000UL);
  
  attachInterrupt(MFB_INTPIN,multiBtn,RISING);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  // !!!Not Callibrated!!!
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (ESP12 GPIO 12)..."));
    attachInterrupt(MPU_INTPIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print("Packet Size: ");
    Serial.println(packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  Serial.println();
  Serial.print("char array :");
  Serial.println(samMaxLength);
}

void loop() {

///---MQTT CONNECTION---///

    if (!client.connected()) {
        reconnect();
    }
    client.loop();

///---DMP CHECKER---///
     
     if (!dmpReady) return;

///---WAIT MPU INTERRUPT/EXTRA PACKET---///

    while (!mpuInterrupt && fifoCount < packetSize) {}

///---MORE MPU ROUTINE---///

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    ///---FIFO OVERFLOW HANDLER---///
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        ////////////////////////////////////////////////////////////////////
        /////               DATA SAMPLING ROUTINE                     //////
        ////////////////////////////////////////////////////////////////////
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        int buffX=map(aaReal.x,-24000,24000,-99,99);
        int buffY=map(aaReal.y,-24000,24000,-99,99);
        int buffZ=map(aaReal.z,-24000,24000,-99,99);
          
        int buffA=map(sqrt(sq(aaReal.x)+sq(aaReal.y)+sq(aaReal.z)),0,24000,0,99);
            
//        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//        int buffX=map(aaWorld.x,-24000,24000,-99,99);
//        int buffY=map(aaWorld.y,-24000,24000,-99,99);
//        int buffZ=map(aaWorld.z,-24000,24000,-99,99);
//       
//        int buffA=map(sqrt(sq(aaWorld.x)+sq(aaWorld.y)+sq(aaWorld.z)),0,24000,0,99);
            
          int buffW=(ypr[0] * 180/M_PI);
          int buffP=(ypr[1] * 180/M_PI);
          int buffR=(ypr[2] * 180/M_PI);

          Serial.print("ypr\t");
          Serial.print(buffW+300);
          Serial.print("\t");
          Serial.print(buffP+300);
          Serial.print("\t");
          Serial.print(buffR+300);
          Serial.print("\t");

          Serial.print("areal\t");
          Serial.print(buffX);
          Serial.print("\t");
          Serial.print(buffY);
          Serial.print("\t");
          Serial.print(buffZ);
          Serial.print("\t");
          Serial.print(buffA-200);
          Serial.print("\t");
          Serial.println(-200);

          addSam('X',buffX);
          addSam('Y',buffY);
          addSam('Z',buffZ);
          addSam('A',buffA);
          addSam('R',buffP);
          addSam('P',buffP);
          addSam('W',buffW);
            
          samCount++;
      }
      
  if(samCount>=samPacketLength){                   //try 5 sec interval     
      LED_ON();
      Serial.print("Publishing... Data length: ");
      Serial.println(samLength);
      for(int saP=0;saP<=samLength+10;saP++){                     //check first 100 char
        Serial.print(samChar[saP]);
      }
      for(int saP=samLength;saP<=samMaxLength;saP++){ //clear higher char array (problem found if not cleared
        samChar[saP]=0;
      }
      Serial.println();
      client.publish(mqtt_topic,samChar,samLength);
      Serial.println("Published..!");
      LED_OFF();
      samCount=0;
      samLength=0;
  }
}
