#include <Arduino.h>

#include "ir_module.h"




//===========================================================================
// http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol#
//===========================================================================
/*
   Interrupt stuff
*/
#define  IR_STARTBIT1  0
#define IR_STARTBIT2  1
#define IR_ADDRL    2
#define IR_ADDRH    3
#define IR_CMD      4
#define IR_CMDN     5




//---------------

volatile IRDATA ir_data;
volatile uint8_t ir_data_ready;    

volatile static uint8_t  ir_status;
volatile static uint8_t bits;
unsigned long period;
volatile unsigned long *ir_command;

//-----------------------------------------
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


// //Use one Routine to handle each group

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
  static unsigned char edge = 0;
  unsigned long ir_period;

  if (digitalRead(IR_PIN) == LOW) {
    ir_period = (micros() - period) / 100;
    period = micros();
    edge = 0;
  } else {
    edge = 1;
  }

  switch (ir_status) {
    case IR_STARTBIT1:
      // Workaround to interrupt on any edge
      if (!edge) {
        edge = 1;
      } else {
        ir_period = (micros() - period) / 100;
        period = micros();
        // 9ms leading pulse burst (16 times the pulse burst length used for a logical data bit)
        if ((ir_period >= 85) && (ir_period < 95) && !ir_data_ready)
          ir_status = IR_STARTBIT2;
        else {
          //Serial.print("Period 1 = ");
          //Serial.println(ir_period);
        }
        edge = 0;
      }
      break;

    case IR_STARTBIT2:
      if (!edge) {
        //  4.5ms space
        if ((ir_period > 40) && (ir_period < 50)) {
          ir_status = IR_CMD;
          edge = 0;
          ir_command = (unsigned long*) &ir_data;
          *ir_command = 0;
          //ir_data_ready = 0;
          bits = 0;
         // digitalWrite(LED1, HIGH);
          //Serial.println("IR_ CMD");
        }
        else {
          ir_status = IR_STARTBIT1;
          //Serial.print("Period 2 = ");
          //Serial.println(ir_period);
          return;
        }
      }
      break;

    case IR_CMD:
      if (!edge) {
        if ((bits < 32) && (ir_period < 30)) {
          ir_command = (unsigned long*) &ir_data;
          if ((ir_period > 8) && (ir_period < 15)) {
            *ir_command >>= 1;
            //Serial.write("0",1);
          }
          else if (ir_period > 16) {
            *ir_command >>= 1;
            *ir_command |= 0x80000000;
            //Serial.write("1",1);
            //Serial.print(ir_period);
          }
          bits++;
        }
        if (bits >= 32) {               // addrL + AddrH + cmd + cmdN = 32 bit
          ir_data_ready = 1;
          ir_status = IR_STARTBIT1;
          //digitalWrite(LED1, LOW);
          //Serial.println("IR got data");
          return;
        }

        if (ir_period > 30) {
          ir_status = IR_STARTBIT1;
          //digitalWrite(LED1, LOW);
          //Serial.println("IR wrong data");
          return;
        }
      }
      break;

  }
}


ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  digitalWrite(13, digitalRead(A0));
}


ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  //digitalWrite(GREEN_LED, digitalRead(IR_PIN));

  static unsigned char edge = 0;
  unsigned long ir_period;

  if (digitalRead(IR_PIN) == LOW) {
    ir_period = (micros() - period) / 100;
    period = micros();
    edge = 0;
  } else {
    edge = 1;
  }



}
//=============================================================================

