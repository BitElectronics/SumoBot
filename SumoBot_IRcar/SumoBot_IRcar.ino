/*
  Author : Vyara Simeonova
  Name : SumoBot  Arduino project
  Ver: 1.0.0
  Second edition - 10-10-17

  Роботът се управлява от IR дистанционно управление използващо протокол NEC
  За управление се използват бутоните с цифри - 2, 4, 5, 6, 8, + и -.
  Подредбата е като на джойстик за съответните посоки.
  С плюс и минус се увеличава скоростта а с 5 се спира.
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
#define DOHIO_LEFT  3
#define DOHIO_RIGHT 2
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
#define BUTT1     11
#define BUTT2     7

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


#define LEFT 0
#define RIGHT 1

/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
int dir;
signed int leftMotorSpeed, rightMotorSpeed;
signed int control_value;
signed int speed;

unsigned int frontL_distance;
unsigned int frontR_distance;
unsigned int left_distance;
unsigned int right_distance;

unsigned int startup_timer;

//=================  CALIBRATE ==========================
unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_min[SENSORS_NR];

//--------- Common ------------
char tmp_str[64];



// prascaler bytes
byte PS_16 = (1 << ADPS2);
//byte PS_32 = (1 << ADPS2) | (1 << ADPS0);
//byte PS_64 = (1 << ADPS2) | (1 << ADPS1);
byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//--------------------------------
int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };


//------------------------------------------------------


//------------------------ Function prototypes ------------
void left_motor_speed(signed int motor_speed);
void right_motor_speed(signed int motor_speed);
void read_position(void);



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

  pciSetup(IR_PIN);


  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  left_motor_speed(0);
  right_motor_speed(0);

  // set sample rate to XXXX X001 => 16 micros readings
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_16;


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

  ir_data.command = 0;
  ir_data_ready = 0;

  speed = 50;

}


//============================================================================
//
// the loop routine runs over and over again forever:
//
//============================================================================
void loop() {
  int adc_value;


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


  //-----------------  Има валидна команда от пулта -------------
  if (ir_data_ready) {
    ir_data_ready = 0;             //  Изпълнение на мелодийка
    tone( BEEP, 2600, 60);

    //--------- Управление --------
    if (ir_data.command == KEY_PLUS)      // скоростта се увеличава с 5
      speed = speed + 5;

    if (ir_data.command == KEY_MINUS)     //  скоростта се намалява с 5
      speed = speed - 5;

    if (ir_data.command == KEY_2) {
      left_motor_speed(speed);          // бутон "2", двата мотора се движат с еднаква скорост (роботът се движи напред)
      right_motor_speed(speed);
    }

    if (ir_data.command == KEY_5 ) {
      left_motor_speed(0);              // бутон "5", двата мотора не се движат (роботът е спрял)
      right_motor_speed(0);
    }

    if (ir_data.command == KEY_8) {
      left_motor_speed(-speed);       // бутон "8", двата мотора се движат с еднаква, минусова скорост  (роботът се движи назад)
      right_motor_speed(-speed);
    }

    if (ir_data.command == KEY_6) {
      left_motor_speed(speed);        // бутон "6", единият мотор се движи, а другият не (роботът завива надясно)
      right_motor_speed(0);
    }

    if (ir_data.command == KEY_4) {
      left_motor_speed(0);            // бутон "4", единият мотор се движи, а другият не (роботът завива наляво)
      right_motor_speed(speed);
    }

  }


  delay(2);


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
  delayMicroseconds(250);    // Wait for lighting
  for (sens = 0; sens < 2; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  sensor_values[DOHIO_RIGHT]  = analogRead(sensors[DOHIO_RIGHT]) / 2;    // Right dohio sensor
  digitalWrite( OPT_ENABLE1, LOW);

  //-------------- Read Right sensors ------------
  digitalWrite( OPT_ENABLE2 , HIGH);
  delayMicroseconds(250);    // Wait for lighting
  sensor_values[DOHIO_LEFT]  = analogRead(sensors[DOHIO_LEFT]) / 2;       // Left dohio sensor
  for (sens = 4; sens < SENSORS_NR; sens++) {
    sensor_values[sens]  = analogRead(sensors[sens]) / 2 -  sensor_min[sens];
  }
  digitalWrite( OPT_ENABLE2, LOW);


}



