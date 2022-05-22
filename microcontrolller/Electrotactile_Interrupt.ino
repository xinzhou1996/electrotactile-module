#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#define BLUEFRUIT_SPI_CS            8
#define BLUEFRUIT_SPI_IRQ           7
#define BLUEFRUIT_SPI_RST           4
#define VERBOSE_MODE                true

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

static uint32_t pattern = 0; //0B00000000111001110011100111000000; //Leftmost 2 bits don't care. Next 5 first row, next 5 second row...
const uint32_t freq_Out = 50; //output frequency 
const uint32_t freq_Mod = 5; // low frequency modulation
static uint8_t serial_index = 0;

static boolean output_enabled = 1; 
static uint32_t outputCounter = 0;
static uint32_t modToggle = freq_Out/freq_Mod;


void setup() {
  Serial.begin(115200);

  PORT->Group[PORTA].DIRSET.reg = 1ul << 2; //Set CLCK as OUTPUT
  PORT->Group[PORTB].DIRSET.reg = 1ul << 8; //Set SERIAL0 as OUTPUT
  PORT->Group[PORTB].DIRSET.reg = 1ul << 9; //Set SERIAL1 as OUTPUT
  PORT->Group[PORTA].DIRSET.reg = 1ul << 4; //Set SERIAL2 as OUTPUT
  PORT->Group[PORTA].DIRSET.reg = 1ul << 5; //Set SERIAL3 as OUTPUT
  PORT->Group[PORTB].DIRSET.reg = 1ul << 2; //Set LATCH_OE as OUTPUT
  PORT->Group[PORTA].DIRSET.reg = 1ul << 11; //Set STRBE as OUTPUT
  PORT->Group[PORTA].DIRSET.reg = 1ul << 10; //Set Shifter_OE as OUTPUT

  //Init:

  PORT->Group[PORTA].OUTCLR.reg = 1ul << 11; //Clear STRBE
  PORT->Group[PORTA].OUTSET.reg = 1ul << 10; //Set Shifter_OE
  PORT->Group[PORTA].OUTSET.reg = 1ul << 2; //Set CLCK
  PORT->Group[PORTB].OUTCLR.reg = 1ul << 2; //Clear LATCH_OE
//  PORT->Group[PORTB].OUTSET.reg = 1ul << 2; //SET LATCH_OE
  

  Serial.print(F("Initialising the Bluefruit LE module: "));

  ble.begin(VERBOSE_MODE);
  Serial.println( F("Done!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    ble.factoryReset();
    ble.sendCommandCheckOK("ATI");
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  ble.verbose(false);

  Serial.println(F("Waiting for device to connect..."));
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    //    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }


  Serial.println( F("DEVICE CONNECTED!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  startTimer(32*freq_Out);
}

void loop() {
  decodeData();
  
  }

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;

  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;

  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Callback:
    if (serial_index == 16) serial_index = 0;

    PORT->Group[PORTA].OUTTGL.reg = 1ul << 2; //toggle CLCK
    switch (serial_index) {

      case 0: //CLOCK LOW
        PORT->Group[PORTA].OUTSET.reg = 1ul << 11; //Set STRBE
        delayMicroseconds(2);
        PORT->Group[PORTA].OUTCLR.reg = 1ul << 11; //Clear STRBE

//TEST START
        outputCounter++;

        if (outputCounter == modToggle){
          outputCounter = 0;
//          PORT->Group[PORTB].OUTTGL.reg = 1ul << 2;  //Toggle LATCH_OE

          
          

          }

       
//TEST END

//PULSE OFF:
        output_enabled = 1;
//        output_enabled = !output_enabled;
        
        // SERIAL0 | PORTB << 8:
        if (pattern >> 27 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 10 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 19 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        break;

      case 2:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 22 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 5 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 24 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 12 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 4:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 21 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 0 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 29 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 8 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 6:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 26 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 11 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 18 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 3 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 8:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 16 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 1 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 28 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 13 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 10:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 25 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 6 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 23 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 4 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 12:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 20 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 7 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL2 | PORTA << 4:
        if (pattern >> 17 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 4;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 4;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 9 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;

      case 14:
        // SERIAL0 | PORTB << 8:
        if (pattern >> 15 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 8;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 8;
        }
        // SERIAL1 | PORTB << 9:
        if (pattern >> 2 & 1ul & output_enabled) {
          PORT->Group[PORTB].OUTSET.reg = 1ul << 9;
        } else {
          PORT->Group[PORTB].OUTCLR.reg = 1ul << 9;
        }
        // SERIAL3 | PORTA << 5:
        if (pattern >> 14 & 1ul & output_enabled) {
          PORT->Group[PORTA].OUTSET.reg = 1ul << 5;
        } else {
          PORT->Group[PORTA].OUTCLR.reg = 1ul << 5;
        }
        break;
    }
    serial_index++;
  }
}

void decodeData() {
  if ( ble.available() )
  {
    int data = ble.read();
    if (data & 4) {
      pattern |= 1ul << (29 - (data >> 3));
    } else {
      pattern &= ~(1ul << (29 - (data >> 3)));
    }
  }
}
