/*
  Author : Simeon Simeonov
  Name : DogBot  Arduino project
  Two PID regulators fpr each wheel
  Ver: 1.0.0
*/
#include <Arduino.h>
#include <PWM.h>


#include "pitches.h"
#include <stdio.h> // for function sprintf

//--------------------- PIN definitions -----------
/*
  #define RIGHT_SIDE  A2
  #define RIGHT_FRONT A3
  #define DOHIO_LEFT  A4
  #define DOHIO_RIGHT A5
  #define LEFT_FRONT  A6
  #define LEFT_SIDE   A7
*/
//---------  Array index -------
#define RIGHT_SIDE  0
#define RIGHT_FRONT 1
#define DOHIO_LEFT  2
#define DOHIO_RIGHT 3
#define LEFT_FRONT  4
#define LEFT_SIDE   5

//----------  Motors --------
#define LEFT_PWM    9
#define RIGHT_PWM   10
#define LEFT_DIR    5
#define RIGHT_DIR   6


//--------------- LEDS -----------
#define LED1      4      // LED1

//-------------- SENSORS ---------
#define OPT_ENABLE1   3
#define OPT_ENABLE2   2

//-------------- Beep ----------
#define BEEP        13

//--------------  Buttons -----------
#define BUTT1     7
#define BUTT2     11

//-------------- Ultrasonic --------
#define TRIG      12
#define ECHO      A0

//----------------- IR --------------
#define IR_PIN    8

//-------------------------- Sensors and position -------------------------
#define SENSORS_NR  6
const unsigned int sensors[SENSORS_NR] = { A2, A3, A4, A5, A6, A7 }; //left-right

//--------------- Калибрираща константа ------------
#define CALLIBRATE  32000


//---------------- Състояние ---------
#define FORWARD    1
#define LEFT       2
#define RIGHT      3
#define BACK       4


///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define PWM_FREQ      50000


//------------------   -------------
#define FRONT_DISTANCE       150     //200mm
#define SIDE_DISTANCE         100     //мм  - Разстояние за обръщане в старани

#define KP             4
#define KD             10
#define Ki           0.03
#define SPEED           110
#define ACQUIRE_SPEED  30



#define ATTACK_SPEED        200
#define COMBAT_SPEED        255
#define TURN_SPEED          220
#define BACK_SPEED          190
#define BREAK_SPEED         250


/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
int dir;
signed int error;
signed int lastError;
signed int prevError;
signed int left_pwm;
signed int right_pwm;
signed int motorSpeed;
signed int control_value;
signed int speed;


unsigned int frontL_distance;
unsigned int frontR_distance;
unsigned int left_distance;
unsigned int right_distance;

signed int left_error;
signed int right_error;
signed int lastLeft_error;
signed int lastRight_error;
signed long leftIntegral = 0;
signed long rightIntegral = 0;

//=================  CALIBRATE ==========================
unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_min[SENSORS_NR];

//--------- Common ------------
char tmp_str[64];

//--------------------------------
int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };



//////////////////////////////////////////////////////////////////////////////
//
//                          SETUP
//////////////////////////////////////////////////////////////////////////////

//     Първоначална инициализация на всички входове, изходи и променливи
//
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //---- Инициализация на пиновете за управление на моторите
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  //---- PWM инициализация за управление на моторите
  InitTimersSafe();
  if ( SetPinFrequencySafe(LEFT_PWM, PWM_FREQ))
  {
    pinMode(LEFT_PWM, OUTPUT);
  }
  if (SetPinFrequencySafe(RIGHT_PWM, PWM_FREQ))
  {
    pinMode(RIGHT_PWM, OUTPUT);
  }

  // Управление на оптроните
  pinMode( OPT_ENABLE1 , OUTPUT);
  pinMode( OPT_ENABLE2 , OUTPUT);
  digitalWrite( OPT_ENABLE1 , LOW);
  digitalWrite( OPT_ENABLE2 , LOW);

  // --- Изходи  ---
  pinMode(LED1, OUTPUT);
  pinMode(BEEP, OUTPUT);

  // --- Входове ---
  pinMode(BUTT1, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);



  tone(BEEP, 2400, 400);
  noTone(BEEP);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  left_motor_speed(0);
  right_motor_speed(0);

  // Изчакване натискането на бутон
  while (digitalRead (BUTT1) == HIGH);
  dir = FORWARD;
  lastError = 0;
  prevError = 0;

  // ---  Изпълняване на мелодията ---
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second
    // divided by the note type. //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BEEP, melody[thisNote], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BEEP);
  }

}


//============================================================================
//
// the loop routine runs over and over again forever:
//
//============================================================================
void loop() {
  int adc_value;
  long quadr;
  int  derivate;

  //---- Прочитане на текущата позиция и пресмятане на грешката ----
  read_position();
  frontL_distance = CALLIBRATE / sensor_values[LEFT_FRONT];
  frontR_distance = CALLIBRATE / sensor_values[RIGHT_FRONT];
  left_distance = CALLIBRATE / sensor_values[LEFT_SIDE];
  right_distance = CALLIBRATE / sensor_values[RIGHT_SIDE];
  /*
      Left and right dohio sensors are analog values.
      Lower than 100 is the white border. Higher - dohio black disk.
      sensor_values[DOHIO_LEFT], sensor_values[DOHIO_RIGHT]
  */

  //error = lAngle - turn;

  //--------------------- Left PID ----------------
  if (  frontL_distance <90)
       frontL_distance = 0;
  left_error =  frontL_distance - FRONT_DISTANCE;

  if ( frontL_distance > 250 ) {
    left_error = 0;
    leftIntegral = 0;
  }

  control_value = KP * left_error + KD * (left_error - lastLeft_error) + Ki * leftIntegral;
  lastLeft_error = left_error;
  leftIntegral += left_error;
  if (leftIntegral > 800)
    leftIntegral = 800;
  if (leftIntegral < -800)
    leftIntegral = -800;

  if (control_value > 200)
    control_value = 200;
  if (control_value < -200)
    control_value = -200;

  left_pwm =  control_value;

  //--------------------- Right PID ---------------------
  if (frontR_distance  <90)
      frontR_distance  = 0;
  right_error = frontR_distance - FRONT_DISTANCE;

  if ( frontR_distance  > 250 ) {
      right_error = 0;
      rightIntegral = 0;
  }
    

  control_value = KP * right_error + KD * (right_error - lastRight_error) + Ki * rightIntegral;
  lastRight_error = right_error;
  rightIntegral += right_error;
  if (rightIntegral > 800)
    rightIntegral = 800;
  if (rightIntegral < -800)
    rightIntegral = -800;

  if (control_value > 200)
    control_value = 200;
  if (control_value < -200)
    control_value = -200;

  right_pwm = control_value;


  //---------------------------------
  if (left_distance <= SIDE_DISTANCE) {   // Враг от ляво
    left_motor_speed(-TURN_SPEED );    // - трябрва да тръгне на обратно - Завой на място
    right_motor_speed(TURN_SPEED);
    delay(150);
  }
  else if (right_distance <= SIDE_DISTANCE) {  // Враг от дясно
    left_motor_speed(TURN_SPEED);
    right_motor_speed(-TURN_SPEED );  // трябрва да тръгне на обратно - Завой на място
    delay(150);
  }


  left_motor_speed(left_pwm);
  right_motor_speed(right_pwm);

  delay(5);

}



//=======================================================================================

/* -----------------------------------------------------------------------------------
    Two way PWM speed control. If speed is >0 then motor runs forward.
    Else if speed is < 0 then motor runs backward
  ------------------------------------------------------------------------------------ */
void left_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
    digitalWrite(LEFT_DIR, HIGH);
  }
  else {
    digitalWrite(LEFT_DIR, LOW);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(LEFT_PWM, motor_speed);
}

//---------------------------------------
void right_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
    digitalWrite(RIGHT_DIR, HIGH);
  }
  else {
    digitalWrite(RIGHT_DIR, LOW);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(RIGHT_PWM, motor_speed);
}



//===================== Read sensors  and scale ==================
void read_position(void) {
  unsigned char sens;
  unsigned int tmp_value;

  for (sens = 0; sens < SENSORS_NR; sens++) {
    sensor_min[sens]  = analogRead(sensors[sens]) / 2;
  }

  //-------------- Read Left sensors ------------
  digitalWrite( OPT_ENABLE1 , HIGH);
  delayMicroseconds(320);    // Wait for lighting
  for (sens = 0; sens < SENSORS_NR / 2; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  digitalWrite( OPT_ENABLE1, LOW);

  //-------------- Read Right sensors ------------
  digitalWrite( OPT_ENABLE2 , HIGH);
  delayMicroseconds(320);    // Wait for lighting
  for (sens = SENSORS_NR / 2; sens < SENSORS_NR; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  digitalWrite( OPT_ENABLE2, LOW);


}

