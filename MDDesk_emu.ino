//Mazda MDdesk emulator based on tapedesk emulator
/*
Adress calc for send from to base unit:
    tapedesk to base unit =>0x00 + 0x08 = 0x08
    CDdesk to BU => 0x03+0x08 => 0x0B
    MDdesk to BU => 0x07+0x08 = 0x0F
Checksum calc formula => (b1^b2^b3)+1)&0x0F
view nikosapi website for more detail
*/
//Configuration
#include "Keyboard.h"
#define ENABLE_DEBUG_OUTPUT

//Depended parameters

#define BIT_LOW_LEVEL_DURATION_MIN  (1400)  //value in us
#define BIT_LOW_LEVEL_DURATION_MAX  (2000)  //value in us

#ifdef ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(...)  if(Serial){ Serial.print(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
#endif

//Constants
#define TYPE_IO_PIN_INPUT_MODE          (INPUT_PULLUP)
#define TYPE_IO_PIN                     2

#define RX_TIMEOUT_MS                   12U
#define IN_BUFFER_SIZE                  96U

#define NIBBLE_RESET_BIT_POS            0x08


/* message mustbe aligned to bytes*/
typedef struct rxMessage {
  uint8_t target;
  uint8_t command;
  uint8_t data[];
} rxMessage_t;

typedef enum rxMessageTarget_e {
  Target_TapeDesk = 0x00,
  Target_Unknown = 0x01,
  Target_CDDesk = 0x03,
  Target_CDChangerExt = 0x05,
  Target_CDChangerUpper = 0x06,
  Target_MDDesk = 0x07,
  Target_BaseUnit = 0x08
} rxMessageTarget_t;

typedef enum rxMessageCommand_e {
  Command_Control = 0x01,
  Command_AnyBodyHome = 0x08,
  Command_WakeUp = 0x09
} rxMessageCommand_t;

typedef enum rxMessageSubCommand_e {
  SubCommand_Playback = 0x01,
  SubCommand_SeekTrack = 0x03,
  SubCommand_SetConfig = 0x04
} rxMessageSubCommand_t;

typedef enum SubConmmandPlayback_e {
  Playback_Play = 0x01,
  Playback_FF = 0x04,
  Playback_REW = 0x08,
  Playback_Stop = 0x60
} SubConmmandPlayback_t;

typedef enum SubCommandSetConfig_e {
  SetConfig_RepeatMode = 0x40, //0x01 for tape desk and 0x40 for disk
  SetConfig_RandomMode = 0x02,
  SetConfig_FastForwarding = 0x10,  //for tape only
  SetConfig_FastRewinding = 0x20,  //for tape only
  SetConfig_ScanMode = 0x08 //for disk only
} SubCommandSetConfig_t;


// data present in nibbles, byte equal nibble
//Wakeup notification
const uint8_t MDCMD_POWER_ON[] =          {0x0F, 0x08, 0x01, 0x07};         //Wake up notification
//Status messages: {target, command(status), arg1, arg2, checksum}
const uint8_t MDCMD_STOPPED[] =           {0x0F, 0x09, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C};   //0 - Stopped, C - not use desk
const uint8_t MDCMD_PLAYING[] =           {0x0F, 0x09, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04};   //4 - Playing, 1 - tape in use
const uint8_t MDCMD_SEEKING[] =           {0x0F, 0x09, 0x05, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01};   //5 - seeking, 1 - tape in use

//Detailed status  {target, command(det. status), arg1, arg2, arg3, arg4, arg5, arg6, checksum}
const uint8_t MDCMD_DISC_PRESENT[] =      {0x0F, 0x0B, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x01};
const uint8_t MDCMD_PLAYBACK[] =          {0x0F, 0x0B, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0E};

const uint8_t MDCMD_FAST_REWIND[] =     {0x0F, 0x09, 0x07, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0};
const uint8_t MDCMD_FAST_FORWARD[] =    {0x0F, 0x09, 0x06, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0};

//disc info
//                                   addr,dinfo,cd0/1,1sttracknbr,endtrachnbr,totalminute,totalsecond,
const uint8_t MDCMD_DISCINFO[] =    {0x0F, 0x0C, 0x01, 0x00, 0x01, 0x00, 0x0F, 0x04, 0x01, 0x03, 0x08, 0x0F, 0x0E};
//unknown message
const uint8_t MDCMD_UNKNOWN[] =     {0x0F, 0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x0C};
//first wakeup message at anybodyhome
const uint8_t MDCMD_FIRSTWAKEUP[] = {0x0F, 0x0B, 0x0A, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0C, 0x05};

static uint8_t inNibblesBuffer[IN_BUFFER_SIZE] = {0U};
static uint8_t nibblesReceived = 0;
static uint8_t biteShiftMask = NIBBLE_RESET_BIT_POS;
static uint32_t rx_time_us = 0;
static uint32_t rx_time_ms = 0;

//##################Checksum calc############################
/* Usage exemple:
    size_t length = sizeof(data) - 1; // Exclure le LRC de la taille totale
    appendLRC(data, length);
*/
uint8_t calculateLRC(uint8_t *data, size_t length) {
    uint8_t lrc = 0;

    for (size_t i = 0; i < length; ++i) {
        lrc ^= data[i];
    }
    lrc = (lrc+1)&0x0F;
    return lrc;
}
void appendLRC(uint8_t *data, size_t length) {
    uint8_t lrc = calculateLRC(data, length);

    // Ajouter le LRC à la fin du tableau
    data[length] = lrc;
}
//################END Checksum #############################
//tracknb,repeatmode,randommmode,
uint8_t playconf[] = {0x00,0x01,0x00,0x00};

//this update cmdarray with current track number, and mode
void appendtracknbr(uint8_t *data, size_t length)
{
  /*uint8_t data2[length];
  memcpy(data2, data, sizeof(data[0])*length);
  //arg3,4 in array is track*/
  data[3]=playconf[0]; //tracknb[0];
  data[4]=playconf[1];  //tracknb[1];
  data[11]=playconf[2]; //rptmode;
  data[12]=playconf[3]; //rdmmode;

  //checksum update
  //size_t length2 = sizeof(data2) - 1; // Exclure le LRC de la taille totale
  appendLRC(data, length-1);

  return data;
}


void setup() {

  pinMode(TYPE_IO_PIN, TYPE_IO_PIN_INPUT_MODE);
  Keyboard.begin();

  attachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN), collectInputData, CHANGE);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.begin(9600);
#endif

  DEBUG_PRINT("Init....\r\n");
  send_message(MDCMD_POWER_ON, sizeof(MDCMD_POWER_ON));
}

void loop() {

  if ( ( millis() - rx_time_ms ) > RX_TIMEOUT_MS) {
    if (nibblesReceived != 0U ) {

      noInterrupts(); {
        DEBUG_PRINT("RX[");
        DEBUG_PRINT(nibblesReceived);
        DEBUG_PRINT("] ");

        for (int i = 0; i < nibblesReceived; i++) {
          DEBUG_PRINT(inNibblesBuffer[i], HEX);
        }
        DEBUG_PRINT("\r\n");

        process_radio_message((rxMessage_t*)inNibblesBuffer);

        bufferReset();

        rx_time_ms = millis();

      } interrupts();
    }
  }
}

void bufferReset() {
  for (uint8_t i = 0U; i < nibblesReceived; i++) {
    inNibblesBuffer[i] = 0U;
  }

  nibblesReceived = 0;
  biteShiftMask = NIBBLE_RESET_BIT_POS;
}

void collectInputData() {
  uint32_t elapsed_time = 0;

  // calculate pulse time
  elapsed_time = micros() - rx_time_us;
  rx_time_us = micros();
  rx_time_ms = millis();

  if (digitalRead(TYPE_IO_PIN) == LOW) {
    return;
  }

  if ( (elapsed_time > BIT_LOW_LEVEL_DURATION_MIN) && (elapsed_time < BIT_LOW_LEVEL_DURATION_MAX) ) {
    inNibblesBuffer[nibblesReceived] |= biteShiftMask;
  }

  biteShiftMask >>= 1U;

  if (biteShiftMask == 0U) {
    biteShiftMask = NIBBLE_RESET_BIT_POS; //save one nibble to one byte
    ++nibblesReceived;
  }

  if (nibblesReceived >= IN_BUFFER_SIZE) {
    DEBUG_PRINT("Buffer overflow, reset!\r\n");
    bufferReset();
  }
}

static void send_nibble(const uint8_t nibble) {
  uint8_t nibbleShiftMask = 0x08;
  uint8_t bit_value = 0U;

  while (nibbleShiftMask != 0U) {
    // Pull the bus down
    digitalWrite(TYPE_IO_PIN, LOW);

    bit_value = nibble & nibbleShiftMask;

    //Set Logic 0 or 1 time
    if (bit_value) {
      delayMicroseconds(1780);
    } else {
      delayMicroseconds(600);
    }

    // Release the bus
    digitalWrite(TYPE_IO_PIN, HIGH);

    //End logic pause
    if (bit_value) {
      delayMicroseconds(1200);
    } else {
      delayMicroseconds(2380);
    }

    nibbleShiftMask >>= 1U;
  }
}

// Send a message on the Mazda radio bus
void send_message(const uint8_t *message, const uint8_t lenght) {
  DEBUG_PRINT("TX[");
  DEBUG_PRINT(lenght);
  DEBUG_PRINT("] ");

  for (int i = 0; i < lenght; i++) {
    DEBUG_PRINT(((uint8_t*)message)[i], HEX);
  }

  DEBUG_PRINT("\r\n");

  noInterrupts(); {

    do {
      delay(10);
    } while (digitalRead(TYPE_IO_PIN) != HIGH);

    detachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN));
    digitalWrite(TYPE_IO_PIN, HIGH);
    pinMode(TYPE_IO_PIN, OUTPUT);

    for (uint8_t i = 0; i < lenght; i++) {
      send_nibble(message[i]);
    }

    pinMode(TYPE_IO_PIN, TYPE_IO_PIN_INPUT_MODE);
    attachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN), collectInputData, CHANGE);

  } interrupts();
}


void process_radio_message(const rxMessage_t *message) {
  //check target, 0 is tape desk
  /*if (message->target != Target_MDDesk || message->target != Target_TapeDesk) {
    return;
  }*/

  switch (message->command) {
    case Command_AnyBodyHome:
      DEBUG_PRINT("Any body home msg\r\n");
      send_message(MDCMD_POWER_ON, sizeof(MDCMD_POWER_ON));
      //delay(8);
      //send_message(MDCMD_FIRSTWAKEUP, sizeof(MDCMD_FIRSTWAKEUP));

      break;
    case Command_WakeUp:
      DEBUG_PRINT("Wake up msg\r\n");

      send_message(MDCMD_DISC_PRESENT, sizeof(MDCMD_DISC_PRESENT));
      delay(8);
      send_message(MDCMD_UNKNOWN, sizeof(MDCMD_UNKNOWN));
      delay(8);
      send_message(MDCMD_DISCINFO, sizeof(MDCMD_DISCINFO));
      //delay(8);
      //send_message(MDCMD_STOPPED, sizeof(MDCMD_STOPPED));
      break;
    case Command_Control:
      if (message->data[0] == SubCommand_Playback) {
        uint8_t subCmd = ((message->data[1] << 4U) & 0xF0) | (message->data[2] & 0x0F);
        if (subCmd == Playback_Play) {
          DEBUG_PRINT("Playback MSG = Playback_Play\r\n");
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   
          delay(8);
          appendtracknbr(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));
          send_message(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));  //append playing message with new track number   
          delay(8);
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   
          //Keyboard.write('X');
        } else if (subCmd == Playback_FF) {
          Keyboard.write('N');
          DEBUG_PRINT("Playback MSG = Playback_FF\r\n");
          //must make function append for track number et timing
          appendtracknbr(MDCMD_FAST_FORWARD, sizeof(MDCMD_FAST_FORWARD));
          send_message(MDCMD_FAST_FORWARD, sizeof(MDCMD_FAST_FORWARD)); //upgrade to follow tracking number?
          delay(8);
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number 

        } else if (subCmd == Playback_REW) {
          Keyboard.write('V');
          DEBUG_PRINT("Playback MSG = Playback_REW\r\n");
          //must make function append for track number et timing
          appendtracknbr(MDCMD_FAST_REWIND, sizeof(MDCMD_FAST_REWIND));
          send_message(MDCMD_FAST_REWIND, sizeof(MDCMD_FAST_REWIND)); //upgrade to follow tracking number?
          delay(8);
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   
        } else if (subCmd == Playback_Stop) {
          DEBUG_PRINT("Playback MSG = Playback_Stop\r\n");
          send_message(MDCMD_STOPPED, sizeof(MDCMD_STOPPED));
          Keyboard.write('C');

        } else {
          DEBUG_PRINT("Playbçack MSG = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else if (message->data[0] == SubCommand_SeekTrack) {  //format is 0KK1 where kk is track number we need to put message into var
        if (message->data[3]==15 && playconf[1]==1){
              Keyboard.write('V');
        } else if (message->data[3]==1 && playconf[1]==15){
          Keyboard.write('N');
        } else if(message->data[3]>playconf[1]){
            Keyboard.write('N');
        } else if (message->data[3]<playconf[1]) {
              //go to previous track
              Keyboard.write('V');
        } 
        
        DEBUG_PRINT("SubCommand_SeekTrack\r\n");
        //store news track number
        //playconf[0] = message->data[2];
        playconf[1] = message->data[3];

        appendtracknbr(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));  //append seeking message with new track number
        send_message(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));
        delay(8);
        appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   
        send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));

      } else if (message->data[0] == SubCommand_SetConfig) {
        uint8_t subCmd = ((message->data[1] << 4U) & 0xF0) | (message->data[2] & 0x0F);
        if ( subCmd == SetConfig_RepeatMode) {
          DEBUG_PRINT("Enable RepeatMode\r\n");
          playconf[2]=0x04; //enable repeat mode
          /*if(rptmode!=0x04) {
            rptmode=0x04;
          } else {
            rptmode=0x00;
          }*/
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number 
          //delay(8);
          //send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number     

        } else if ( subCmd == SetConfig_RandomMode) {
          DEBUG_PRINT("Enable Random\r\n");
          playconf[3]=0x02; //enable random
          /*if(rdmmode!=0x02) {
            rdmmode=0x02;
          } else {
            rdmmode=0x00;
          }*/
          appendtracknbr(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));  //append seeking message with new track number
          send_message(MDCMD_SEEKING, sizeof(MDCMD_SEEKING));
          delay(8);
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   
          //delay(8);
          //send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number   

        } else if ( subCmd == SetConfig_FastForwarding) {
          DEBUG_PRINT("SetConfig_FastForwarding\r\n");
          Keyboard.write('N');
          send_message(MDCMD_FAST_FORWARD, sizeof(MDCMD_FAST_FORWARD));
          delay(8);
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));

        } else if ( subCmd == SetConfig_FastRewinding ) {
          DEBUG_PRINT("SetConfig_FastRewinding\r\n");
          Keyboard.write('V');
          send_message(MDCMD_FAST_REWIND, sizeof(MDCMD_FAST_REWIND));
          delay(8);
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));   
          
        } else if ( subCmd == 0x00 ) {
          DEBUG_PRINT("Disable RepeatRandom\r\n");
          playconf[2]=0x00; //disable repeat
          playconf[3]=0x00; //disable random
          /*rptmode=0x00;
          rdmmode=0x00;*/
          appendtracknbr(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));
          send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number 
          //delay(8);
          //send_message(MDCMD_PLAYING, sizeof(MDCMD_PLAYING));  //append playing message with new track number     
        } else {
          DEBUG_PRINT("SubCommand_SetConfig = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else {
        DEBUG_PRINT("UNCKNOWN Sub command\r\n");
      }
      break;
    default:
      DEBUG_PRINT("another cmd = ");
      DEBUG_PRINT(message->command);
      DEBUG_PRINT("\r\n");
      break;
  }
}

void send_byte( uint8_t c ) {
  c = ~c;

  digitalWrite(TYPE_IO_PIN, LOW);
  for ( uint8_t i = 10; i; i-- ) {            // 10 bits
    delayMicroseconds( 100 );                 // bit duration
    if ( c & 1 ) {
      digitalWrite(TYPE_IO_PIN, LOW);
    } else {
      digitalWrite(TYPE_IO_PIN, HIGH);
    }
    c >>= 1;
  }
  delayMicroseconds(10); //30 Wait at least 10 us between bytes
}
