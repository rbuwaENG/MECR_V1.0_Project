
/*
███╗░░░███╗███████╗░█████╗░██████╗░░█████╗░██████╗░░█████╗░░░░██╗░░░██╗░░░░░███╗░░░░░░█████╗░
████╗░████║██╔════╝██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗░░░██║░░░██║░░░░████║░░░░░██╔══██╗
██╔████╔██║█████╗░░██║░░╚═╝██████╔╝██║░░██║██████╦╝██║░░██║░░░╚██╗░██╔╝░░░██╔██║░░░░░██║░░██║
██║╚██╔╝██║██╔══╝░░██║░░██╗██╔══██╗██║░░██║██╔══██╗██║░░██║░░░░╚████╔╝░░░░╚═╝██║░░░░░██║░░██║
██║░╚═╝░██║███████╗╚█████╔╝██║░░██║╚█████╔╝██████╦╝╚█████╔╝██╗░░╚██╔╝░░██╗███████╗██╗╚█████╔╝
╚═╝░░░░░╚═╝╚══════╝░╚════╝░╚═╝░░╚═╝░╚════╝░╚═════╝░░╚════╝░╚═╝░░░╚═╝░░░╚═╝╚══════╝╚═╝░╚════╝░
DATE:20/5/2023
Programme:Buwaneka R.I*/

//servo lid
#include <ESP32Servo.h>
Servo lidservo;
#define pos 20
int stat= 0;
#define trigPinlid 14
#define echoPinlid 12
long duration;
volatile int distance;

//bluetooth
#include "BluetoothSerial.h";
BluetoothSerial SerialBT;
int BTData;
String command = "";
#define bled 2 //blutooth led

//motor a
#define IN1  21
#define IN2  19


//motor b
#define IN3 18
#define IN4  5

//obstacle avoiding
#define S1Trig 26
#define S1Echo 27
#define S2Trig 33
#define S2Echo 25
#define S3Trig 35
#define S3Echo 32
int obstacleStat = 0;



//path Memorize
int farry[50];
int barry[50];
int larry[50];
int rarry[50];

int startPressed = 0;    // the moment the button was pressed
int endPressed = 0;      // the moment the button was released
int fholdTime =0;
int bholdTime =0;
int lholdTime =0;
int rholdTime =0;
int pathIndex =0;

//speed
//NOT IMPLEMENTED IN TIS VERSION
int speedM = 150;

void setup() {
  //servo lid setup
  lidservo.attach(13);
  pinMode(trigPinlid,OUTPUT);
  pinMode(echoPinlid,INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPinlid),handleInterrupt,CHANGE);
  
  //Motor setup
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  //Avoid obstacles setup
  pinMode(S1Trig,OUTPUT);
  pinMode(S1Echo,INPUT);
  pinMode(S2Trig,OUTPUT);
  pinMode(S2Echo,INPUT);
  pinMode(S3Trig,OUTPUT);
  pinMode(S3Echo,INPUT);

  //bluetooth setup
  SerialBT.begin("MEC_ROBOT");
  Serial.begin(115200);
  pinMode(bled,OUTPUT);

  
}

void loop() {
  //bluetooth recive commands
  if(SerialBT.available()>0){ 
    command = SerialBT.readStringUntil('\n');
    Serial.println(command);
    //blutooth indicator
    digitalWrite(bled, HIGH);
  }
  else{
    digitalWrite(bled, LOW);
  }
  //speed 
  if(command.startsWith("s")){
    speedMotors();
    Serial.println(speedM);
    command ="";
    }
  
  //forward
  if(command == "UP"){
    startPressed = millis();
    fholdTime  = startPressed - endPressed;
    forward();
    Serial.println("forward");
    endPressed =0;
    
    }
  else if(command =="STOP"){
    endPressed = millis();
    stopM();
    command = "";
    startPressed =0;
   
    }

  //backward
   if(command == "DOWN"){
    startPressed = millis();
    bholdTime  = startPressed - endPressed;
    backward();
    Serial.println("backward");
    endPressed =0;
    
    }
  else if(command == "STOP"){
    endPressed = millis();
    stopM();
    command = "";
    startPressed =0;
   
    }
    //left
   if(command == "LEFT"){
    startPressed = millis();
    left();
    lholdTime  = startPressed - endPressed;
    Serial.println("left");
    endPressed =0;
    
    }
   else if(command == "STOP"){
    endPressed = millis();
    stopM();
    command = "";
    startPressed =0;
   
    }

    //right
   if(command == "RIGHT"){
    startPressed = millis();
    rholdTime  = startPressed - endPressed;
    right();
    Serial.println("right");
    endPressed =0;
    
    }
  else if(command == "STOP"){
    endPressed = millis();
    stopM();
    command = "";
    startPressed =0;
   
    }
    //Serial.println(fholdTime);
    //Serial.println(bholdTime);
  //lidcontrol
  if(command == "A"){
    lidcontrol();
    command ="";
    }

  if(command == "B"){
    if(obstacleStat == 0){
      obstacleStat++;
      Serial.println("obstacleavoid:turnon");}
     else {
      obstacleStat--;
      Serial.println("obstacleavoid:turnoff");}
    command ="";}

  //main programe
  if(command == "SAVE"){
    savefunc();
    command ="";}

  else if(command == "RUN"){
    runfunc();
    command ="";
    }

  if(command == "RESET"){
    resetfunc();
    command ="";
    }
    
  //autolid open
   lidcontrolauto();
  

  
}
//speed 
void speedMotors(){
  String speedofM = command.substring(1,command.length());
  speedM = speedofM.toInt();
  }

//path memorize function
void savefunc(){
  //save the movement
  farry[pathIndex] = fholdTime/10;
  barry[pathIndex] = bholdTime/10;
  larry[pathIndex] = lholdTime/10;
  rarry[pathIndex] = rholdTime/10;
  pathIndex++;
  command ="";
  Serial.println(pathIndex);
  fholdTime = 0;
  bholdTime = 0;
  lholdTime = 0;
  rholdTime = 0;
}

void runfunc(){
  
  for(int i =0;i<pathIndex;i++){
    if(farry[i]> 0){
      forward();
      Serial.println("forward");
      Serial.println(farry[i]);
      delay(farry[i]);
      stopM();
      Serial.println("Stop");
      command ="";
    }
    if(barry[i]> 0){
      backward();
      Serial.println("backward");
      Serial.println(barry[i]);
      delay(barry[i]);
      stopM();
      Serial.println("Stop");
      command ="";
    }
   if(larry[i]> 0){
      left();
      Serial.println("left");
      Serial.println(larry[i]);
      delay(larry[i]);
      stopM();
      Serial.println("Stop");
      command ="";
    }
   if(rarry[i]> 0){
      right();
      Serial.println("right");
      Serial.println(rarry[i]);
      delay(rarry[i]);
      stopM();
      Serial.println("Stop");
      command ="";
    }
  }
  
 
}

void resetfunc(){
  memset(farry, 0, sizeof(farry));
  memset(barry, 0, sizeof(barry));
  memset(larry, 0, sizeof(larry));
  memset(rarry, 0, sizeof(rarry));
  pathIndex =0;
}

//Motor functions 
void forward(){
  avoidObstacles(1);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  
  bholdTime =0;
  lholdTime =0;
  rholdTime =0;

  
}

void backward(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  
  fholdTime =0;
  lholdTime =0;
  rholdTime =0;
  
}

void right(){
  avoidObstacles(3);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);

  fholdTime =0;
  bholdTime =0;
  lholdTime =0;
 

  
}

void left(){
  avoidObstacles(2);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  
  fholdTime =0;
  bholdTime =0;
  rholdTime =0;

  
}

void stopM(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}


void lidcontrol(){
  if(stat ==1){lidservo.write(pos);stat--;
  Serial.println("lidclose");
  delay(1000);
  }
  else if(stat ==0){lidservo.write(pos+160);stat++;
  Serial.println("lidopen");
  delay(1000);
  }

  }

void lidcontrolauto(){
  digitalWrite(trigPinlid,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinlid,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinlid,LOW);

  
  }

void handleInterrupt(){
  static unsigned long startTime = 0;
  static unsigned long endTime = 0;
  static bool isStarted =false;
  
  if(digitalRead(echoPinlid)==HIGH){
    startTime = micros();
    isStarted = true;}
   else if(isStarted){
    endTime =micros();
    isStarted = false;
    distance = (endTime-startTime)/58;
   if(distance < 5){
      lidservo.write(pos+160);
      Serial.println("lidopen");
      Serial.println(distance);
      delay(1000);
    }
  else{
      lidservo.write(pos);
      Serial.println("lidclose");
      delay(1000);}}
    
    }

void avoidObstacles(int stat){
  if(obstacleStat == 1){
    bool center = sensorOne();
    bool left = sensorTwo();
    bool right = sensorThree();
    if(stat ==1){
      if(center == true){
        stopM();}
    }
    else if(stat==2){
      if(left == true){
        stopM();}
      }
    else if(stat ==3){
       if(right == true){
        stopM();}
      }
      
     }
  

    }
      
  
  
  

bool sensorOne(){
  digitalWrite(S1Trig,LOW);
  delayMicroseconds(4);
  digitalWrite(S1Trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(S1Trig,LOW);

  long d = pulseIn(S1Echo,HIGH);
  int cm = d*17;
  bool obstaclefound = false;
  if(cm <= 20){
    obstaclefound = true;}
  else {
    obstaclefound = false;
    }
    
  Serial.println(cm);
  return obstaclefound;
  
  }
  
bool sensorTwo(){
  delayMicroseconds(4);
  digitalWrite(S2Trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(S2Trig,LOW);

  long d = pulseIn(S2Echo,HIGH);
  int cm = d*17;
  bool obstaclefound = false;
  if(cm <= 20){
    obstaclefound = true;}
  else {
    obstaclefound = false;
    }
    
  Serial.println(cm);
  return obstaclefound;
  }

bool sensorThree(){
  delayMicroseconds(4);
  digitalWrite(S3Trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(S3Trig,LOW);

  long d = pulseIn(S3Echo,HIGH);
  int cm = d*17;
  bool obstaclefound = false;
  if(cm <= 20){
    obstaclefound = true;}
  else {
    obstaclefound = false;
    }
    
  Serial.println(cm);
  return obstaclefound;
  
  }
