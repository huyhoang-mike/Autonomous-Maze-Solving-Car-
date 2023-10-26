#include <PID_v1.h>
#include <NewPing.h>
// SONA Sensors
#define TRIGL  A2  // Arduino pin tied to trigger pin on ping sensor.
#define ECHOL  A3  // Arduino pin tied to echo pin on ping sensor.

#define TRIGR  A0  // Arduino pin tied to trigger pin on ping sensor.
#define ECHOR  A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGF1  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHOF1  A5  // Arduino pin tied to echo pin on ping sensor.

#define L_left 11 // black
#define L_right 12 // white
#define button1 9 // red
#define button2 10 // brown
boolean mode = 0;
boolean start = 0;
int nut1ST, nut2ST ;

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters).


NewPing sonarLeft(TRIGL, ECHOL, MAX_DISTANCE);
NewPing sonarRight(TRIGR, ECHOR, MAX_DISTANCE);
NewPing sonarFront1(TRIGF1, ECHOF1, MAX_DISTANCE);

float T = 14  ; // chuẩn
float T1 = 8;  // giảm lắc góc vuông

unsigned int pingSpeed = T/2; // How frequently are we going to send out a ping (in milliseconds).
unsigned long pingTimer;     // Holds the next ping time.
float oldLeftSensor,oldRightSensor,
      leftSensor,rightSensor,
      frontSensor1,oldFrontSensor1,
      lSensor,rSensor,
      fSensor1,fSensor2;
float pre_leftSensor ;
float pre_rightSensor ;
float pre_frontSensor1 ;
//=============================================================== wall_threshold
int wall_threshold = 20 ; //20
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 8 ;//8
int chongcham1 = 6;//5
int chongcham2 = 4;
int ket =0;
boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;
boolean PreWall[3] = {0,0,0};

// Motor Variables              
int motorInput1 = 3;      
int motorInput2 = 4;
int motorInput3 = 7;      
int motorInput4 = 8;                 
int ENA = 6;  //Motor Phải
int ENB = 5;  //Motor Trái

//Initial Speed of Motor
int initial_motor_speed = 80; // 80
int motor_speed = initial_motor_speed ;

//================================================================ Tốc độ rẽ
int banh_chinh1 = 87;//95-70-87
int banh_phu1 = 37; //Đảo ngược-110-130-37
int banh_chinh2 = 87;//95-70-87
int banh_phu2 = 35; //Đảo ngược-35
int banh_chinh;//95-70
int banh_phu ; //Đảo ngược-110-130
//vọt lố - tăng phụ, giảm chính
//chưa tới - tăng chính giảm phụ

//================================================================ PID Constants
double Kp = 1.8;//1.8
double Ki = 0.0;//-0.02-0.04-00
double Kd = 0.15;//0.1-0.15

double pre_error = 0;
double error = 0, PID_value = 0, Setpoint = 0;
int max_PID_value = 255 - motor_speed + 100;

//Khai báo PID
  PID myPID(&error, &PID_value, &Setpoint, Kp, Ki, Kd, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                                //P_ON_E (Proportional on Error) is the default behavior
//timer
unsigned long time1;
unsigned long time2;
unsigned long time3;
unsigned long time4;

void setup() {
  // put your setup code here, to run once:
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(L_left, OUTPUT);
  pinMode(L_right, OUTPUT);
//timer
  time1 = millis();
  time2 = millis();
  time3 = millis();
  time4 = millis();
  
  Serial.begin(115200);              
  delay(500);
  Serial.println("Started !!");
  delay(100); 
  
  myPID.SetOutputLimits(-max_PID_value, max_PID_value);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(T);//-12-13
}
//================================================================ LOOP

void loop() {
  // put your main code here, to run repeatedly:
  ReadSensors();
  Wall();
  if ((PreWall[0] == leftwall) && (PreWall[1] == rightwall) && (PreWall[2] == frontwall)&& (ket == 0) ) {
    time4 = millis();
    ket = 1;
  }else if ((PreWall[0] == leftwall) && (PreWall[1] == rightwall) && (PreWall[2] == frontwall)&& (ket == 1)&&((unsigned long) (millis() - time4) >= 8000)) {
    chong_ket();
    ket = 0;
  }else if ((frontSensor1 < chongcham1) || (leftSensor < chongcham2) || (rightSensor < chongcham2)){
  chong_cham();
  }else LeftWallFollow(); 
// read sensors & print the result to the serial monitor //

  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor1);
  Serial.println(" cm ");
  Serial.print("PID=");
  Serial.println(PID_value);
  PreWall[0] = leftwall;
  PreWall[1] = rightwall;
  PreWall[2] = frontwall;
}

void ReadSensors() {

  lSensor = sonarLeft.ping_cm();//ping in cm
    if(lSensor==0) lSensor = MAX_DISTANCE;
  rSensor = sonarRight.ping_cm();
    if(rSensor==0) rSensor = MAX_DISTANCE;
  fSensor1 = sonarFront1.ping_cm();
    if(fSensor1==0) fSensor1 = MAX_DISTANCE;

  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2 ;
  frontSensor1 = (fSensor1 + oldFrontSensor1) / 2;

  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor1 = frontSensor1;
}
void Wall() {
if (leftSensor <= wall_threshold) leftwall = 1;
  else leftwall = 0;
if (rightSensor <= wall_threshold) rightwall = 1;
  else rightwall = 0;
if (frontSensor1 <= front_threshold) frontwall = 1;
  else frontwall = 0;
if ((leftwall == 1) &&  (rightwall == 1) && (frontwall == 0)) { //110
  error = rightSensor - leftSensor;
  pre_error = error;
  }
else if ((leftwall == 1) &&  (rightwall == 0) && (frontwall == 0)) { //100
  error = (16-leftSensor) - leftSensor;
  pre_error = error;
  }
else if ((leftwall == 0) &&  (rightwall == 1) && (frontwall == 0)) { //010
  error = rightSensor - (28-rightSensor) ;
  pre_error = error;
  }
else error = pre_error;
}
//================================================================LeftWallFollow - MODE 0
void LeftWallFollow() {
if ((leftwall == 1) && (rightwall == 1) && (frontwall == 1)){
  ReadSensors();
  Wall();
  PreWall[0] = leftwall;
  PreWall[1] = rightwall;
  PreWall[2] = frontwall;
  time1 = millis();
  do{
  ReadSensors();
  Wall();
  TurnBack();
  } while(((unsigned long) (millis() - time1) <= 350) || PreWall[2] == frontwall);
  time2 = millis();
}
else if (leftwall == 0) {
  ReadSensors();
  Wall();
  PreWall[0] = leftwall;
  PreWall[1] = rightwall;
  PreWall[2] = frontwall;
  chonspeed ();
  time3 = millis();
  do{
  ReadSensors();
  Wall();
  if ((unsigned long) (millis() - time3) >  2000) chong_ket_trai ();
  chong_cham();
  sharpLeftTurn();
  } while((PreWall[0] == leftwall) && (PreWall[1] == rightwall)&& (PreWall[2] == frontwall));
  time2 = millis();
}
  else if (frontwall == 1) {
  ReadSensors();
  Wall();
  PreWall[0] = leftwall;
  PreWall[1] = rightwall;
  PreWall[2] = frontwall;
  chonspeed ();
  time3 = millis();
  do{
  ReadSensors();
  Wall();
  if ((unsigned long) (millis() - time3) >  2000) chong_ket_phai ();
  chong_cham();
  sharpRightTurn();
  }while((PreWall[0] == leftwall) && (PreWall[1] == rightwall));
  time2 = millis();
  }else {
    chong_cham();
    myPID.Compute();
    motor_control();
  }
}

void motor_control()
{
  // Calculating the effective motor speed:
  
  int left_motor_speed = motor_speed - PID_value ;   //
  int right_motor_speed = motor_speed + PID_value;  //

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);   //
  right_motor_speed = constrain(right_motor_speed, 0, 255); //

  analogWrite(ENB, left_motor_speed); //Left Motor Speed      
  analogWrite(ENA, right_motor_speed); //Right Motor Speed  

  //following lines of code are to make the bot move forward
  forward();
}
//================================================================CHỐNG CHẠM
void chong_cham(){
  //chong_ket();
  if (frontSensor1 < chongcham1) {
    ReadSensors();
    do {
      ReadSensors();
      analogWrite(ENA, 70);        //R Motor Speed
      digitalWrite(motorInput1, LOW);
      digitalWrite(motorInput2, HIGH);
      analogWrite(ENB, 70);         //L Motor Speed
      digitalWrite(motorInput3, LOW);
      digitalWrite(motorInput4, HIGH);
    } while(frontSensor1 < (chongcham1 +0.5));
  } else if (leftSensor < chongcham2) {
    ReadSensors();
    do {
      ReadSensors();
      analogWrite(ENA, 75);        //R Motor Speed
      digitalWrite(motorInput1, HIGH);
      digitalWrite(motorInput2, LOW);
      analogWrite(ENB, 50);         //L Motor Speed
      digitalWrite(motorInput3, LOW);
      digitalWrite(motorInput4, HIGH);
    } while(leftSensor < (chongcham2 + 0.5));
  }else if (rightSensor < chongcham2) {
    ReadSensors();
    do {
      ReadSensors();
      analogWrite(ENA, 50);        //R Motor Speed
      digitalWrite(motorInput1, LOW);
      digitalWrite(motorInput2, HIGH);
      analogWrite(ENB, 75);         //L Motor Speed
      digitalWrite(motorInput3, HIGH);
      digitalWrite(motorInput4, LOW);
    } while(rightSensor < (chongcham2 +0.5));
  }
}

void forward()
{
    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW); //LOW = TIẾN
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
}
void Stop()
{
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, LOW); //LOW = TIẾN
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, LOW);
}
//================================================================CHỌN SPEED
void chonspeed (){
  if ((unsigned long) (millis() - time2) <= 500) { //600
    banh_chinh = banh_chinh1;
    banh_phu = banh_phu1;
  } else {
    banh_chinh = banh_chinh2;
    banh_phu = banh_phu2;
  }
}

//================================================================TỐC RẼ
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENA, banh_chinh);        //R Motor Speed
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  analogWrite(ENB, banh_phu);         //L Motor Speed
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENA, banh_phu );        //R Motor Speed
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  analogWrite(ENB, banh_chinh);         //L Motor Speed
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void TurnBack() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENB, 50);       //Bánh trái tiến
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  analogWrite(ENA, 90);       //bánh phải lùi
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

void Revert() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENB, 90);       //Bánh trái tiến
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  analogWrite(ENA, 90);       //bánh phải lùi
  digitalWrite(motorInput3, LOW );
  digitalWrite(motorInput4, HIGH);
}

void Revertleft() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENB, 90);       //Bánh trái tiến
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  analogWrite(ENA, 70);       //bánh phải lùi
  digitalWrite(motorInput3, HIGH );
  digitalWrite(motorInput4, LOW);
}

void Revertright() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(ENB, 70);       //Bánh trái tiến
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  analogWrite(ENA, 90);       //bánh phải lùi
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

//================================================================CHỐNG KẸT
void chong_ket() {
    ReadSensors();
    Wall();
    time1 = millis();
  do {
    ReadSensors();
    Wall();
    //chong_cham();
    Revert();
  }while((unsigned long) (millis() - time1) < 300);
}
void chong_ket_phai() {
    ReadSensors();
    Wall();
    time1 = millis();
  do {
    ReadSensors();
    Wall();
    //chong_cham();
    Revertright();
  }while((unsigned long) (millis() - time1) < 200);
}

void chong_ket_trai() {
    ReadSensors();
    Wall();
    time1 = millis();
  do {
    ReadSensors();
    Wall();
    //chong_cham();
    Revertleft();
  }while((unsigned long) (millis() - time1) < 200);
}
