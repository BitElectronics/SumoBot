

#ifndef _IR_MODULE_
#define _IR_MODULE_

//-----------------------------------------------------
#define IR_PIN    8

//-------------- IR remote control --------------------
#define  IR_NEC_REMOTE1
#define  IR_CMD_START   0x40    // KEY NEXT
#define  IR_CMD_STOP    0x46    // KEY CH


#ifdef IR_NEC_REMOTE1
//----------------- NEC commands ----------
#define    KEY_1   0x0C
#define   KEY_2   0x18
#define   KEY_3   0x5E
#define   KEY_4   0x08
#define   KEY_5   0x1C
#define   KEY_6   0x5A
#define   KEY_7   0x42
#define   KEY_8   0x52
#define   KEY_9   0x4A
#define   KEY_0   0x16
#define   KEY_100   0x19
#define   KEY_200   0x0D
#define   KEY_PREV  0x44
#define   KEY_NEXT  0x40
#define   KEY_PLAY  0x43
#define   KEY_CHUP  0x47
#define   KEY_CHDN  0x45
#define   KEY_CH    0x46
#define   KEY_PLUS  0x15
#define   KEY_MINUS 0x07
#define   KEY_EQ    0x09
#endif


#ifdef IR_NEC_REMOTE2
//----------------- NEC commands ----------
#define   KEY_1   0x16
#define   KEY_2   0x19
#define   KEY_3   0x0D
#define   KEY_4   0x0C
#define   KEY_5   0x18
#define   KEY_6   0x5E
#define   KEY_7   0x08
#define   KEY_8   0x1C
#define   KEY_9   0x5A
#define   KEY_0   0x52
#define   KEY_PREV  0x44
#define   KEY_NEXT  0x43
#define   KEY_PLAY  0x40
#define   KEY_UP    0x46
#define   KEY_DN    0x15
#define   KEY_STAR  0x42
#define   KEY_SHARP 0x4A

#define   KEY_CH    0x4A  //  #
#define   KEY_MINUS 0x42  //  *
#define   KEY_EQ    0x40  //  OK
#endif


void pciSetup(byte pin);


typedef struct {
  unsigned char address_low;
  unsigned char address_high;
  unsigned char command;
  unsigned char command_n;
} IRDATA;


/*
   Variables
*/
extern volatile IRDATA ir_data;

extern volatile  uint8_t ir_data_ready;    //  Флаг, който показва че име вече приета команда. Трябва след прочитане да се нулира

//------------------------------------------------------

#endif
