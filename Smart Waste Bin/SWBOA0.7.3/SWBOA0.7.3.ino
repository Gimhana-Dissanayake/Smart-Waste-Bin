#define trig A0
#define echo A1
#define trig1 A4
#define echo1 A5

#define tilt 2

#include <Servo.h>

#include<SoftwareSerial.h>
SoftwareSerial mySerial(22, 23);

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(12,13);//CNS, CE

Servo servoBinMotor;
Servo servoCapMotor;

const byte address[6] = "00001";//pipe that will be used by the two transeiver modules for communication
                                //Used to specify which receiver we will be talking with

//Right motor
int enR = 9;//EnB
int inR1 = 5;//N3
int inR2 = 4;//N4

//Left motor
int enL = 3;//EnA
int inL1 = 7;//N2
int inL2 = 6;//N1

int photocellPin = A3;
int photocellPin1 = A2;
int pCP1 = A6;
int pCP2 = A7;
int photocellReading; 
int photocellReading1;
int pCR1;
int pCR2;


int unload = 0;
long duration,cm,proximinity,threshold,duration1;
int mode = 0;

int tiltSensed;

char fullText[5];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  servoBinMotor.attach(10);
  servoCapMotor.attach(11);

  pinMode(tilt, INPUT);
  
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);

  radio.begin();//Initialize the radio obj
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);//Power amplifier level;The two modules are close by so it is min
  radio.stopListening();//this function sets this transeiver module as the transmitter
  
  servoInitial();
  
}

int maxPower = 90;

void loop() {

  if(mode == 0)
  {

  collisionDetection();//Malaka Component

  proximinity = getDistance();//Isuru Component
  Serial.print("proximinity ");
  Serial.println(proximinity);
  while(proximinity == 0)
  {   
  proximinity = getDistance(); 
  } 
    
  if(proximinity < 5)
  {
  openCap();  
  delay(5000);      
  } 

   threshold = checkThreshold();
   while(threshold == 0)
   {
   threshold = checkThreshold();
   }
    
   if(threshold < 8)
   {
  fullText[0] = 'B';
  fullText[1] = '0';
  fullText[2] = '1';
  fullText[3] = 'F';
   radio.write(&fullText, sizeof(fullText));//sizeof function is used to specify the number of bytes needed to be taken from the fullText variable to the &fullText variable
   mode++;
   }
  Serial.print("Threshold ");
  Serial.println(threshold);
  
  }else if(mode == 1)
  {
   initiateWasteDisposalProcess();
   
  }else{}

  
  }

  void mPower(int m1, int m2)
  {
  if(m1>0)
  {
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  }else
  {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  m1 = m1 * -1;
  }
  analogWrite(enL, m1);

  if(m2>0)
  {
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
  }else
  {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  m2 = m2 * -1;
  }
  analogWrite(enR, m2);
  }


  void stopMotor()
  {//Stops the motors
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  delay(1000);  
  }

  void servoInitial()
  {
  servoBinMotor.write(10);
  servoCapMotor.write(0);
  delay(1000);
  }

  long getDistance()
  {//Obstacle avoidance functionality
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH, 100000);
  return (duration / 2) / 29.1;
  }

  long checkThreshold()
  {//Obstacle avoidance functionality
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  duration1 = pulseIn(echo1, HIGH, 100000);
  return (duration1 / 2) / 29.1;
  }

  void openCap()
  {
  servoCapMotor.write(90);
  delay(7000);
  servoCapMotor.write(0);
  delay(500);    
  }

  void wasteUnload()
  {
    
 // stopMotor();    
  servoCapMotor.write(90);
  
  for(int i=10; i<=130; i++)
  {
  servoBinMotor.write(i);
  delay(20);
  }

  delay(5000);
    
  for(int j=130; j>=10;j--)
  {
  servoBinMotor.write(j);
  delay(20);
  }

  servoCapMotor.write(0);
    
  }

 void initiateWasteDisposalProcess()
  {
    
  cm = getDistance();
  
  if(unload==0){
  if(cm < 8)
  {
      
  tone(24, 1000, 1000);
  delay(1000);
  stopMotor();
    
  }
  
  photocellReading = analogRead(photocellPin);  
  photocellReading1 = analogRead(photocellPin1);
  
  if(photocellReading>990 && photocellReading1>990)
  {   
    stopMotor();
    wasteUnload();
    unload++;
  }else 
  {
  
  int mL = 0;
  int mR = 0;

  if(photocellReading < 760){
  mL = maxPower;
  mR = -maxPower;
  }else if(photocellReading1 < 760){
  mL = -maxPower;
  mR = maxPower;   
  }else{
  mL = maxPower;
  mR = maxPower;
  }

  mPower(mL, mR);
  
  }
  }else{
    
  pCR1 = analogRead(pCP1);  
  pCR2 = analogRead(pCP2);

  if(pCR1>990 && pCR2>990)
  {
   stopMotor();
  fullText[0] = 'B';
  fullText[1] = '0';
  fullText[2] = '1';
  fullText[3] = 'D';
  radio.write(&fullText, sizeof(fullText));
  mode--;
  return;
  }else{

  int mL = 0;
  int mR = 0;

  if(pCR1 < 740){
  mL = -maxPower;
  mR = maxPower;
  }else if(pCR2 < 740){
  mL = maxPower;
  mR = -maxPower;   
  }else{
  mL = -maxPower;
  mR = -maxPower;
  }

  mPower(mL, mR);
 
  }
  }    
  }
  
  void collisionDetection()//Malaka Component
  {
    
  tiltSensed = digitalRead(tilt);
  if(tiltSensed == HIGH){

    tone(24, 1000);
    sendMessage();
    
  }

  while(tiltSensed == HIGH){

  tiltSensed = digitalRead(tilt);
  stopMotor();
  tone(24, 1000);

  
  }
  noTone(24);
  }

  void sendMessage(){

    mySerial.println("AT+CMGF=1");
    delay(1000);
    mySerial.println("AT+CMGS=\"+94772108922\"\r");
    delay(1000);
    mySerial.println("Bin 105 is Down");
    delay(100);
    mySerial.println((char)26);
    delay(1000);
  }
