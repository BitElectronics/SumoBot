/*
  Author : Vyara Simeonova
  Name : SumoBot  Arduino project
  Ver: 1.1
  Simple sumo programm
*/
#include <Arduino.h>
#include <PWM.h>


#include "pitches.h"
#include <stdio.h> // for function sprintf

#include "ir_module.h"


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
#define DOHIO_RIGHT 2
#define  DOHIO_LEFT 3
#define LEFT_FRONT  4
#define LEFT_SIDE   5

//----------  Motors --------
#define LEFT_PWM    10
#define RIGHT_PWM   9
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

//------------------------ Калибрираща константа -------------------------
// Използва се за линеаризиране и калибриране на сенозрите
//  преобразужането е : distance = CALLIBRATE / sensor_value; - разстоянието е в мм
#define CALLIBRATE  32000


///////////////////////////////////////////////////////////////////////////
//-----------------------------------     ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define PWM_FREQ      50000

//------------------   -------------
#define SIDE_DISTANCE      200     //mm  - Разстояние за обръщане в старани
#define FRONT_DISTANCE     350     //мм  - Прихващане противник отпред
#define FRONT_ATTACK       200    //мм   - Атака - врагът е близо

#define SEARCH_SPEED_HIGH   220
#define SEARCH_SPEED_LOW    80

#define BATTLE_SPEED      255
#define ATTACK_SPEED      200
#define TURN_SPEED        200
#define BACK_SPEED        200

#define DOHIO_BORDER_LEVEL  160


//----------------- IR START /STOP ------------
#define IR_CMD_START    0x40
#define IR_CMD_STOP     0x46

/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
int dir;
signed int motorSpeed;
signed int control_value;
signed int speed;


unsigned int frontL_distance;
unsigned int frontR_distance;
unsigned int left_distance;
unsigned int right_distance;

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

  //--- IR module setup  ---
  pciSetup(IR_PIN);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  left_motor_speed(0);
  right_motor_speed(0);

  tone(BEEP, 2400, 400);
  delay(500);
  noTone(BEEP);

  // Изчакване натискането на бутон
  while (digitalRead (BUTT1) == HIGH);

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

  // ----- Старт от бутон или от IR дистанционно -----
  while (digitalRead (BUTT1) == HIGH) {
    if (ir_data_ready) {
      ir_data_ready = 0;
      if (ir_data.command == IR_CMD_START)
        break;
    }
  }

  //------------ Бърз ход до центъра на дохиото --------
  left_motor_speed(BATTLE_SPEED );
  right_motor_speed(BATTLE_SPEED);
  delay(500);

}


//============================================================================
//
// the loop routine runs over and over again forever:
//
//============================================================================
void loop() {
  int adc_value;

  //---- Прочитане на текущата позиция и пресмятане на разстоянията ----
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

  //------------------------------------- Control -------------------------------
  // -----  Край на дохиото  -----
  if ((sensor_values[DOHIO_LEFT] < DOHIO_BORDER_LEVEL) || (sensor_values[DOHIO_RIGHT]) < DOHIO_BORDER_LEVEL) {
    //   Назад
    left_motor_speed(-BACK_SPEED);
    right_motor_speed(-BACK_SPEED);
    delay(300);       // Растоянието и ъгъла на завъртане се контролират с времето - delay(...)
    // Завъртане
    left_motor_speed( TURN_SPEED);
    right_motor_speed( -TURN_SPEED);
    delay(280);
  }


  //---------------------------------
  if (left_distance <= SIDE_DISTANCE) {   // Враг от ляво - завъртане
    left_motor_speed(-TURN_SPEED);
    right_motor_speed(TURN_SPEED);
    delay(100);                       // ъгълът се определя с времето
  }
  else if (right_distance <= SIDE_DISTANCE) {  // Враг от дясно - завъртане
    left_motor_speed(TURN_SPEED);
    right_motor_speed(-TURN_SPEED);   // ъгълът се определя с времето
    delay(100);
  }
  else if (frontL_distance <= FRONT_DISTANCE && frontR_distance > FRONT_DISTANCE) {
    // Враг напред в ляво - лек завой наляво
    left_motor_speed(ATTACK_SPEED);
    right_motor_speed(ATTACK_SPEED + 50);
  }
  else if (frontR_distance <= FRONT_DISTANCE && frontL_distance > FRONT_DISTANCE) {
    // Враг напред в дясно - лек завой надясно
    left_motor_speed(ATTACK_SPEED + 50);
    right_motor_speed(ATTACK_SPEED);

  } else if (frontR_distance < FRONT_ATTACK || frontL_distance < FRONT_ATTACK) {
    // И двата предни сензора отчитат враг на малко разстояние
    // --- АТАКААААА ---
    left_motor_speed(BATTLE_SPEED);
    right_motor_speed(BATTLE_SPEED);
  } else {
    left_motor_speed(SEARCH_SPEED_HIGH);
    right_motor_speed(SEARCH_SPEED_LOW);
  }


  //----------- Проверка за  команда "СТОП" ---------
  if (ir_data_ready) {
    ir_data_ready = 0;
    if (ir_data.command == IR_CMD_STOP) {
      left_motor_speed(0);
      right_motor_speed(0);
      while (1);
    }
  }


  //while (digitalRead(IR_PIN) == LOW);    // Спиране ако има команда от Старт модул

  delay(3);

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
  for (sens = 0; sens < 2; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  sensor_values[DOHIO_RIGHT]  = analogRead(sensors[DOHIO_RIGHT]) / 2;    // Right dohio sensor
  digitalWrite( OPT_ENABLE1, LOW);

  //-------------- Read Right sensors ------------
  digitalWrite( OPT_ENABLE2 , HIGH);
  delayMicroseconds(320);    // Wait for lighting
  sensor_values[DOHIO_LEFT]  = analogRead(sensors[DOHIO_LEFT]) / 2;       // Left dohio sensor
  for (sens = 4; sens < SENSORS_NR; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  digitalWrite( OPT_ENABLE2, LOW);

}

