#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define intervalLoc 20000
int scanTime = 1; //In seconds
String locationName="";
String manuData="";
boolean locationReady=false;
int ixL;
char tempLP[3];


char ssid[] = "FALTO";//type your ssid
char password[] = "12345678";//type your password

#define mqtt_server "hantamsurga.net"
#define mqtt_port 49877
#define device_name "FALTO_01"
#define mqtt_topic_data_acc "FALTO_01/sensor/acc"
#define mqtt_topic_data_gyro "FALTO_01/sensor/gyro"
#define mqtt_topic_callibration "FALTO_01/sensor/callib"
#define mqtt_topic_callibration_gyro "FALTO_01/sensor/callib/gyro"
#define mqtt_topic_callibration_acc "FALTO_01/sensor/callib/acc"
#define mqtt_topic_data_acc_unit "FALOT_01/sensor/acc/unit"
#define mqtt_topic_data_gyro_unit "FALOT_01/sensor/gyro/unit"

WiFiClient espClient;
PubSubClient client(espClient);

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_R 19
#define LED_G 18
#define LED_B 17

#define REG_EN 16
#define BTN 21

//#define SDA 22
//#define SCL 23

//#define SDA 21
//#define SCL 22

//#define SDA 26
//#define SCL 27

const int MPU_addr=0x68;  // I2C address of the MPU-6050

#define btn_stat digitalRead(BTN)
#define power_on digitalWrite(REG_EN,HIGH)
#define power_off digitalWrite(REG_EN,LOW)


#define BAT_LV 32
#define read_battery analogRead(BAT_LV)

#define battery_lv_callibration_adc 2500 
#define battery_lv_callibration_voltage 3.86 
#define battery_voltage_empty 3.60
#define battery_voltage_full 4.20

float battery_voltage_multiplier;
float battery_voltage;

float read_battery_voltage(){
  return read_battery*battery_voltage_multiplier;
}

float battery_percentage_multiplier;
float battery_percentage;

float read_battery_percentage(){
  float tempRBP=(read_battery_voltage()-battery_voltage_empty)*battery_percentage_multiplier;
  if(tempRBP<0)tempRBP=0;
  if(tempRBP>100)tempRBP=100;
  return tempRBP;
}

#define dPL 20    //data packet length per publish

volatile float dB[7];   //data buffer
volatile uint16_t dBPt=0;    //data buffer pointer
volatile uint32_t nDP=1;     //n data packet
volatile uint16_t dataRaw;
volatile uint8_t ledLV=0;
volatile byte DND=0;
String message_acc;
String message_gyro;
String unit_acc, unit_gyro;

boolean sendLocation;
unsigned long lastReconnectAttempt = 0;
unsigned long sampleTiming=0;
unsigned long timingLoc=0;
byte countLD;
#define sampleInterval 100;


// define LED
#define LED_PIN 2             //Onboard Active-low LED
#define PWR_PIN 16            //Pin connected to regulator's EN
#define MFB_INTPIN 13         

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String pesan = "";
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  
    pesan += (char) payload[i];
  }
  Serial.println();


  // compare the topic to the topic subscribe
  if (strcmp(topic,mqtt_topic_callibration) == 0) {
    // check the pesan
    if(pesan == "walking"){
      // call walkingCalibration
      callibration_data(); 
    } else if (pesan == "stand") {
      callibration_data();
    } else if (pesan == "sit") {
      // call sit Calibration
      callibration_data();
    } else if (pesan == "terlentang") {
      // call terlentang 
      callibration_data();
    } 
  }
  //sendLocation=true;
}

/**
  Publish data to MQTT in easy way
**/
char dataPublish[50];
void publishMQTT(char* topics, String data){
   data.toCharArray(dataPublish, data.length() + 1);
   client.publish(topics, dataPublish);
}

void initPins(){
  pinMode(REG_EN,INPUT_PULLUP);
  pinMode(BTN,INPUT);
  pinMode(BAT_LV,INPUT);
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
}

void initVars(){
  battery_voltage_multiplier = battery_lv_callibration_voltage/battery_lv_callibration_adc;
  battery_percentage_multiplier = 100.00/(battery_voltage_full-battery_voltage_empty);
  sampleTiming = 0;sendLocation=false;
  lastReconnectAttempt = 0;
}

void setLed(byte rs,byte gs, byte bs){
  digitalWrite(LED_R,rs);digitalWrite(LED_G,gs);digitalWrite(LED_B,bs);
}

void setNet() {
  delay(10);
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  byte rotatL=0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    rotatL++;
    //byte rotaC=rotatL%3;
    //setLed((rotaC==0),(rotaC==1),(rotaC==2));
    LED_BLINK(50,200,3);
    delay(500);
  }
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

boolean reconnect() {
  if (client.connect(device_name)) {
    //if (client.connect(device_name,mqtt_user,mqtt_password)) {
    client.subscribe(mqtt_topic_callibration);
  }
  return client.connected();
}

void clientRun() {
  // Loop until we're reconnected
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
    Serial.println("Device not connecting...");
    LED_BLINK(50,100,3);
  } else {
    // Client connected
    //Serial.println("MQTT Connect...");
    sampleRun();
    client.loop();
  }
}

/**
  this procedure to sent accelerometer and gyroscope to mqtt broker

  call this on loop()
**/
void sampleRun(){
    if(sampleTiming<=millis()){
      sampleTiming=millis()+sampleInterval;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   
      dB[0]=float(ax)/200;
      dB[1]=float(ay)/200;
      dB[2]=float(az)/200;
      float aa = sqrt((sq(dB[0])+sq(dB[1])+sq(dB[2]))/3);
      dB[3]= aa;
      dB[4]=float(gx)/200;
      dB[5]=float(gy)/200;
      dB[6]=float(gz)/200;
      float rms_gyro = sqrt((sq(dB[4])+sq(dB[5])+sq(dB[6]))/3);
      dB[7] = rms_gyro;
  
      if(1){    
        Serial.print(dB[0]); Serial.print("\t");
        Serial.print(dB[1]); Serial.print("\t");
        Serial.print(dB[2]); Serial.print("\t");
        Serial.print(dB[3]); Serial.print("\t");
        Serial.print(dB[4]); Serial.print("\t");
        Serial.print(dB[5]); Serial.print("\t");
        Serial.println(dB[6]); Serial.print("\t");
        Serial.println(dB[7]);
      }

      // unit sending
      unit_acc = String(dB[0],2)+":"+String(dB[1],2)+":"+String(dB[2],2)+":"+String(dB[3],2)+":";
      char buff_unit[unit_acc.length()+1];
      message_acc.toCharArray(buff_unit,unit_acc.length()+1);
      client.publish(mqtt_topic_data_acc_unit,buff_unit);
      
      unit_gyro = String(dB[4],2)+":"+String(dB[5],2)+":"+String(dB[6],2)+":"+String(dB[7],2)+":";
      char buff_gyro_unit[unit_gyro.length()+1];
      message_gyro.toCharArray(buff_gyro_unit,unit_gyro.length()+1);
      client.publish(mqtt_topic_data_gyro_unit,buff_gyro_unit);
    
      // buffering the sensor data
      // split the sensor to 2 variable
      message_acc+=(String(dB[0],2)+":"+String(dB[1],2)+":"+String(dB[2],2)+":"+String(dB[3],2)+":");
      message_gyro += (String(dB[4],2)+":"+String(dB[5],2)+":"+String(dB[6],2)+":"+ String(dB[7],2)+":");
      LED_OFF();
    
      // dPL is maximal length of 1 sampling. current : 20
      if(dBPt == dPL){
        Serial.println("Increment : " + String(dBPt));
        Serial.println("Sending data ...");
        // message_acc.replace(" ","");
        // this is for accelerometer 
        char bufD[message_acc.length()+1];
        message_acc.toCharArray(bufD,message_acc.length()+1);
        client.publish(mqtt_topic_data_acc,bufD);
         
        message_acc="";
        dBPt=0;

        // gyro message
         //message_gyro.replace(" ","");
        char bufD_gyro[message_gyro.length()+1];
        message_gyro.toCharArray(bufD_gyro,message_gyro.length()+1);
        client.publish(mqtt_topic_data_gyro,bufD_gyro);
         
        message_gyro="";
        dBPt=0;
        LED_ON();  // notif the LED_ON
      }

      dBPt++; // increment
  }
}


void read_MPU_data(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  

  dB[0]=float(ax)/200;
  dB[1]=float(ay)/200;
  dB[2]=float(az)/200;
  float aa = sqrt((sq(dB[0])+sq(dB[1])+sq(dB[2]))/3);
  dB[3]= aa;
  dB[4]=float(gx)/200;
  dB[5]=float(gy)/200;
  dB[6]=float(gz)/200;
  float rms_gyro = sqrt((sq(dB[4])+sq(dB[5])+sq(dB[6]))/3);
  dB[7] = rms_gyro;
}

void callibration_data(){
  int count = 0;
  String data_acc = " " , data_gyro = " ";
  LED_ON(); // keep LED ON;
  
  // buffer until 20
  while (count < 20) {
    read_MPU_data();
    data_acc += (String(dB[0],2)+":"+String(dB[1],2)+":"+String(dB[2],2)+":"+String(dB[3],2)+":");
    data_gyro += (String(dB[4],2)+":"+String(dB[5],2)+":"+String(dB[6],2)+":"+String(dB[7],2)+":");
    count++;
    Serial.println(count);
    delay(100);
  }

  Serial.println(data_acc); 
  Serial.println(data_gyro);

  char bufD[data_acc.length()+1];
  data_acc.toCharArray(bufD,data_acc.length()+1);
  client.publish(mqtt_topic_callibration_acc,bufD);
  
  char bufD_gyro[data_gyro.length()+1];
  data_gyro.toCharArray(bufD_gyro,data_gyro.length()+1);
  client.publish(mqtt_topic_callibration_gyro,bufD_gyro);
  //publishMQTT(mqtt_topic_data_acc , data_acc);
  //publishMQTT(mqtt_topic_data_gyro , data_gyro);
  LED_OFF();
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

void setup() {
  // put your setup code here, to run once:
  delay(200);                   //push time before device really turned on
  PWR_INIT();
  LED_INIT();
  LED(500);
  Serial.print("Battery Voltage: ");
  Serial.print(read_battery_voltage());

  attachInterrupt(MFB_INTPIN,multiBtn,RISING);

  //delay(100);
  //power_on;
  //initPins();
  //Wire.begin(SDA,SCL);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPUconnection successful" : "MPUconnection failed");
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  setNet();
}

void loop() {
  clientRun();
}
