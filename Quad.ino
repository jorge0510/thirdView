//----------------------------------------------LIBRARIES-------------------------------------------------
#include <Servo.h>
#include <NewPing.h>
#include <TinyGPS.h>
//-------------------------------------------GYRO---------------------------------------------------------
#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
//----------------------------------------------VARIABLES-------------------------------------------------
Servo leftmotor;
Servo rightmotor;
Servo frontmotor;
Servo backmotor;
//-----------------------------------------------SONARES-------------------------------------------------
#define SONAR_NUM     5 
#define MAX_DISTANCE 200 
#define PING_INTERVAL 35 
unsigned long pingTimer[SONAR_NUM]; 
unsigned int cm[SONAR_NUM];        
uint8_t activeSensor = 0;          

NewPing sonar[SONAR_NUM] = {     
//1-LEFT ULTRASONIC(52, 53) 2-RIGHT ULTRASONIC(50, 51) 3-HEAD ULTRASONIC(46, 47) 4-BACK ULTRASONIC(42, 43) 5-BUTTOM ULTRASONIC(38,39)
  NewPing(52, 53, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(46, 47, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE),
  NewPing(38, 39, MAX_DISTANCE),
};


//--------------------------------------------Variables Originales-----------------------------------------


char input[16];
int index = 0;

int mf = 0;
int mr = 0;
int ml = 0;
int mb = 0;

int mf1 = 0;
int mr1 = 0;
int ml1 = 0;
int mb1 = 0;

char f[4];
char r[4];
char b[4];
char l[4];


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  leftmotor.attach(3);
  rightmotor.attach(4);
  frontmotor.attach(2);
  backmotor.attach(5);
  //---------------------------------------------GYRO SETUP--------------------------------------------------
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(); // join I2C bus 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
}
void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if(Serial1.available() > 0) {
    Serial1.readBytesUntil('p', input, 16);
    //Serial.println(input);
    
    f[0] = input[0];
    f[1] = input[1];
    f[2] = input[2];
    f[3] = '\0';
    mf1 = atoi(f);
    if(mf1 != 0){
      mf = mf1;
    }
    r[0] = input[4];
    r[1] = input[5];
    r[2] = input[6];
    r[3] = '\0';
    mr1 = atoi(r);
    if(mr1 != 0){
      mr = mr1;
    }
    b[0] = input[8];
    b[1] = input[9];
    b[2] = input[10];
    b[3] = '\0';
    mb1 = atoi(b);
    if(mb1 != 0){
      mb = mb1;
    }
    l[0] = input[12];
    l[1] = input[13];
    l[2] = input[14];
    l[3] = '\0';
    ml1 = atoi(l);
    if(ml1 != 0){
      ml = ml1;
    }
    if(sonar[0] < 50){
      mf = mf - (mf*30)/100;
      mr = mr - (mr*30)/100;
      ml = ml + (ml*30)/100;    
      mb = mb + (mb*30)/100;  
  }
    if(sonar[1] < 50){
      mf = mf + (mf*30)/100;
      mr = mr + (mr*30)/100;
      ml = ml - (ml*30)/100;    
      mb = mb - (mb*30)/100;  
    }
    if(sonar[2] < 50){
      mf = mf + (mf*30)/100;
      mr = mr - (mr*30)/100;
      ml = ml + (ml*30)/100;    
      mb = mb - (mb*30)/100;        
    }
    if(sonar[3] < 50){
      mf = mf - (mf*30)/100;
      mr = mr + (mr*30)/100;
      ml = ml - (ml*30)/100;    
      mb = mb + (mb*30)/100;  
    }
    if(sonar[4] < 50){
      mf = mf + (mf*30)/100;
      mr = mr + (mr*30)/100;
      ml = ml + (ml*30)/100;    
      mb = mb + (mb*30)/100;
    }
    
    if(mf < 10)mf=10;
    if(mr < 10)mr=10;
    if(mb < 10)mb=10;
    if(ml < 10)ml=10;
    
    frontmotor.write(mf);
    rightmotor.write(mr);
    backmotor.write(mb);
    leftmotor.write(ml);   
   
    memset(input, '\0', 16);
  }else{
    frontmotor.write(10);
    rightmotor.write(10);
    backmotor.write(10);
    leftmotor.write(10);
  }
  Serial.print("Motores:");
  Serial.print(mf);
  Serial.print(",");
  Serial.print(mr);
  Serial.print(",");
  Serial.print(mb);
  Serial.print(",");
  Serial.println(ml);
  Serial.print("Ace:");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print("   ");
  Serial.print("Gyro:");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.println(gz);
  Serial.print("Sonares:");
  Serial.print(sonar[0]);
  Serial.print(",");
  Serial.print(sonar[1]);
  Serial.print(",");
  Serial.print(sonar[2]);
  Serial.print(",");
  Serial.print(sonar[3]);
  Serial.print(",");
  Serial.println(sonar[4]);
  
  //Serial.println(Serial2.read());
}

