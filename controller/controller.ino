/*
    This file is part of "GPIB Adapter", an Arduino based controller for GPIB (IEEE 488).
    Copyright (C) 2018 Toby Thain, toby@telegraphics.com.au

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
 * Signals

 * Connector
 * 1  DIO1   white
 * 2  ..     orange / black stripe
 * 3  ..     pink
 * 4  DIO4   brown / black stripe
 * 5  EOI    black / grey stripe
 * 6  DAV    black
 * 7  NRFD   yellow / black stripe
 * 8  NDAC   yellow
 * 9  IFC    green / grey stripe
 * 10 SRQ    green
 * 11 ATN    brown / grey stripe
 * 12 SHIELD brown
 * 13 DIO5   beige / black stripe
 * 14 ..     grey
 * 15 ..     violet / black stripe
 * 16 DIO8   violet
 * 17 REN    orange / grey stripe
 * 18        orange
 * 19        red / grey stripe
 * 20        red
 * 21        aqua
 * 22        green / black stripe
 * 23        blue / grey stripe
 * 24 GROUND  blue

 * DC     Direction Control -- tied Low -- means that this device is a controller
 * SC     System Controller -- tied High -- means that REN and IFC are transmitted by this controller
 * 
 * Directions
 *   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 | PC5 . PC4 . PC3 . PC2 . PC1 . PC0 |   Port/Pin
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  | REN | IFC | NDAC| NRFD| DAV | SRQ |   Bus
 *   +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
 *   | OUT | OUT | *** | OUT | OUT | OUT | OUT | OUT | OUT | OUT | OUT | OUT | OUT | OUT | IN  | IN  | OUT | IN  |   Dir: Talk (TE High/DC Low)
 *                           \ PE=H: Totem-Pole Output; PE=L: Open Collector Output
 *   | OUT | OUT | *** | OUT | IN  | IN  | IN  | IN  | IN  | IN  | IN  | IN  | OUT | OUT | OUT | OUT | IN  | IN  |   Dir: Listen (TE Low/DC Low)
 *   
 *   TE  DC  ATN EOI***
 *   --- --- --- -------
 *   H   X   H   T (OUT)
 *   L   X   H   R (IN)
 *   X   H   L   R (IN)   Does not occur, because DC is tied Low
 *   X   L   L   T (OUT)
 *  
 * Useful commands:
 *  *ESR?   read Standard Event Status Register
 *  *STB? or serial poll     read Status Byte Register
 *  ID?
 *  *IDN?
 *  HORIZONTAL?    return horizontal settings
 *  CH1?      return vertical settings for Ch 1
 *  
 *  Measured data transceiver output: 0.19V low, 3.30V high   (3.16V after both transceivers installed)
 *  control transceiver measures 3.16V high for totem-pole outputs, 2.80V high for the open collector (SRQ, NRFD, NDAC)
 */

#include "controller.h"

#define PB_OUTPUTS    0b110100 // PORTB Pins that are always outputs
#define PB_BIDI_FWD   0b001011 // PORTB Bidirectional pins that follow TALK ENABLE (transmit when TE is high)
#define PC_OUTPUTS    0b110000 // PORTC Pins that are always outputs
#define PC_BIDI_FWD   0b000010 // PORTC Bidirectional pins that follow TALK ENABLE
#define PD_BIDI_FWD 0b11111100 // PORTD Bidirectional pins that follow TALK ENABLE

// Address of instrument. In my case, Tektronix TDS460A.
// Check GPIB address on the Shift-STATUS screen, I/O tab.

#define MY_SCOPE 2


void mode(bool talk, byte atn_eoi_pe){
  // SAFETY: Arduino outputs connected to transceiver lines configured as outputs are
  //         likely to violate driver current limits. Set all Arduino pins to high-Z,
  //         so that none are outputs when transceiver lines are switched to outputs by
  //         the change in Talk Enable line.
  //           As high-Z pins, these will be pulled up or down according to
  //         the wired resistors on the terminal side (GPIO pin). 
  //         All are pulled up to Vcc except NRFD and NDAC which are pulled down to GND.

  // Set all bidirectional pins to INPUT
          
  DDRB = PB_OUTPUTS;
  DDRC = PC_OUTPUTS;
  DDRD = 0; // all port D pins are bidirectional

  if(talk) {
    // DIO1-8, EOI, DAV are becoming outputs.
    // Since pins are configured as inputs (above), setting them HIGH enables
    // Arduino's internal pull-ups. (Redundant as I have external pull-ups.)
    PORTC |= PC_BIDI_FWD;
    PORTD  = PD_BIDI_FWD;
    PORTB  = PB_TE_MASK | atn_eoi_pe; //--------- Set transceiver to TALK ---------
    
    // Now set actual direction to match transceiver/TE
    DDRC = PC_OUTPUTS | PC_BIDI_FWD;
    DDRD = PD_BIDI_FWD;
    DDRB = PB_OUTPUTS | PB_BIDI_FWD;
  } else {
    // NRFD, NDAC, EOI (depending on ATN) are becoming outputs
    PORTC &= ~PC_NRFD_MASK; // set NRFD LOW (true on GPIB)
    PORTC &= ~PC_NDAC_MASK; // set NDAC LOW (true on GPIB)
    PORTB  = atn_eoi_pe; //--------- Set transceiver to LISTEN ---------
    
    // Now set actual direction to match transceiver/TE
    DDRC |= PC_NRFD_MASK;
    DDRC |= PC_NDAC_MASK;

    // EOI follows direction of Talk Enable, UNLESS ATN is LOW when it is the opposite of Talk Enable.
    if(!(atn_eoi_pe & PB_ATN_MASK)) {
      PORTB |= PB_EOI_MASK;
      DDRB  |= PB_EOI_MASK; // EOI is output
    }
  }
}

#define TRANSMIT_TIMEOUT_MS 500

volatile unsigned millisCountdown, millisCountUp;

byte transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return ERR;
  }

  PORTC |= PC_DAV_MASK; // data not valid
  
  if((PINC & (PC_NRFD_MASK | PC_NDAC_MASK)) == (PC_NRFD_MASK | PC_NDAC_MASK)) {
    return ERR;
  } else {
#ifdef DEBUG
    char b[9];
    eight_bits(value, b);

    Serial.print("transmit: ");
    Serial.print(b);
    b[0] = b[2] = ' ';
    b[1] = value >= ' ' && value < 0x7f ? value : ' ';
    b[3] = 0;
    Serial.print(b);
    
    if(!(mask & PB_ATN_FALSE_MASK)) Serial.print("  ATN");
    if(!(mask & PB_EOI_FALSE_MASK)) Serial.print("  EOI");
    Serial.println(mask & PB_PE_TOTEMPOLE ? "  PE(TP)" : "  ~PE(OC)");
#endif
    value = ~ value;
    PORTB = PB_TE_MASK | mask | (value >> 6);
    PORTD = value << 2;
    delayMicroseconds(2);
    
    // Wait until all acceptors are ready for data
    countdown(TRANSMIT_TIMEOUT_MS);
    while(!(PINC & PC_NRFD_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NRFD was not raised within 500ms. Aborting transmit");
      return TIMEOUT;
    }
      
    PORTC &= ~PC_DAV_MASK; // data valid
    
    // Wait until all acceptors have accepted the data
    countdown(TRANSMIT_TIMEOUT_MS);
    while(!(PINC & PC_NDAC_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NDAC was not raised within 500ms. Aborting transmit");
      return TIMEOUT;
    }
    
    PORTC |= PC_DAV_MASK; // data not valid
    return SUCCESS;
  }
}

byte receive(byte *value){
  if(PORTB & PB_TE_MASK) { // interface must be in Listen direction
    return ERR;
  }

  PORTC &= ~PC_NDAC_MASK; // not accepted data
  PORTC |= PC_NRFD_MASK;  // ready for data

  // wait for data valid to go low (true)
  countdown(5000);
  while((PINC & PC_DAV_MASK) && millisCountdown)
    ;

  if(millisCountdown == 0) {
    Serial.println("Data not valid within 5 sec. Aborting receive");
    return TIMEOUT;
  }

  PORTC &= ~PC_NRFD_MASK; // not ready for data
  
  *value = ~((PINB << 6) | (PIND >> 2));
  byte res = PINB & PB_EOI_MASK ? SUCCESS : EOI; // remembering level -> logic inversion

  PORTC |= PC_NDAC_MASK; // accepted data
  
  return res;
}

byte tx_data(const char str[]) {
  mode(TE_TALK, DATA_NO_EOI);
  for(const char *p = str; *p; ++p) {
    byte res = transmit(p[1] ? DATA_NO_EOI : DATA_EOI, *p);
    if(res != SUCCESS) {
      return res;
    }
  }
  return SUCCESS;
}

byte rx_data(byte *buf, size_t max, size_t *count, bool is_binary) {
  // This might be called in a block loop, so don't bother
  // changing direction unless necessary.
  if(PORTB & PB_TE_MASK) mode(TE_LISTEN, DATA_NO_EOI);
  
  size_t n = 0;
  for(byte *p = buf; n < max; ++n, ++p) {
    byte res = receive(p);
    if(res == EOI || (!is_binary && *p == '\n')) {
      *count = ++n;
      return SUCCESS;
    } else if(res != SUCCESS) {
      return res;
    }
  }
  *count = n;
  return BUFFER_FULL;
}

#define RECEIVE_BUFFER_SIZE 0x200

byte cmd(byte msg) {
  mode(TE_TALK, CMD_NO_EOI);
#ifdef DEBUG
  Serial.print("cmd:  ");
#endif
  return transmit(CMD_NO_EOI, msg);
}

void serialpoll() {
  byte stat;
  char bits[9];
  
  Serial.println("Serial poll");
  cmd(MSG_UNLISTEN);
  cmd(MSG_SER_POLL_ENB);
  cmd(MSG_TALK | MY_SCOPE);
  mode(TE_LISTEN, DATA_NO_EOI);
  receive(&stat);
  cmd(MSG_SER_POLL_DIS);
  Serial.print("Status byte: ");
  eight_bits(stat, bits);
  Serial.print(bits);
  Serial.println();
}

bool check_srq() {
  if(!(PINC & PC_SRQ_MASK)) {
    serialpoll();
  }
  return (PINC & PC_SRQ_MASK) == PC_SRQ_MASK;
}

bool check(byte res) {
  switch(res) {
    case EOI:
    case SUCCESS:
      return 1;
    case ERR:
      Serial.println("Transmit error (NRFD & NDAC)\n"); 
      return 0;
    case TIMEOUT: 
      Serial.println("Timeout\n"); 
      return 0;
  }
  return 0;
}

void eight_bits(byte v, char *s) {
  for(byte m = 0x80; m; m >>= 1) {
    *s++ = v & m ? '1' : '0';
  }
  *s = 0;
}

ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0; // reset timer
  if(millisCountdown) --millisCountdown;
  ++millisCountUp;
}

void countdown(unsigned ms) {
  TIMSK1 = 0;
  millisCountdown = ms;
  TIMSK1 = (1 << OCIE1A);
}

#define TEXT   0
#define BINARY 1

bool send_query(byte device, const char *command, bool binary_mode, byte *buf, size_t sz) {
  static char base64[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  Serial.print("?> ");
  Serial.println(command);
  
  bool ok = check_srq() &&
            check(cmd(MSG_UNLISTEN))        && check_srq() &&
            check(cmd(MSG_LISTEN | device)) && check_srq() &&
            check(tx_data(command))         && check_srq() &&
            check(cmd(MSG_UNLISTEN))        && check_srq() &&
            check(cmd(MSG_UNTALK))          && check_srq() &&
            check(cmd(MSG_TALK | device))   && check_srq();
  if(ok) {
    size_t n = 0, rx_total = 0, groups = 0;
    byte res;

    millisCountUp = 0;
    
    if(binary_mode) {
      sz -= (sz % 3) + 3; // make buffer a multiple of 3 for correct Base64 encoding
      Serial.println("%%% Base64 data:");
    }

    do {
      res = rx_data(buf, sz, &n, binary_mode);
      if(res == SUCCESS || res == BUFFER_FULL) {
        rx_total += n;

        if(binary_mode) {
          size_t i;
          buf[n] = buf[n+1] = buf[n+2] = 0; // zero out trailing octet group after defined data
          for(i = 0; i < n; i += 3) {
            byte n1 = buf[i] >> 2;
            byte n2 = (buf[i] << 4) | (buf[i+1] >> 4);
            byte n3 = (buf[i+1] << 2) | (buf[i+2] >> 6);
            byte n4 = buf[i+2];
            Serial.print(                 base64[n1]           ); // Always defined
            Serial.print(                 base64[n2 & 0b111111]); // Always defined
            Serial.print(i >= n-1 ? '=' : base64[n3 & 0b111111]); // Only defined if we have data for p[1] and p[2]
            Serial.print(i >= n-2 ? '=' : base64[n4 & 0b111111]); // Only defined if we have data for p[2]
            ++groups;
            if(groups == 18) {
              Serial.print('\n');
              groups = 0;
            }
          }
        } else {
          buf[n] = 0;
          for(size_t i = 0; i < n; ++i) if(buf[i] == '\r') buf[i] = '\n';
          Serial.print((char*)buf);
        }
      } else {
        return 0;
      }
    } while(res == BUFFER_FULL);
    if(binary_mode) Serial.println("\n%%% Base64 end");
    
    Serial.print("  Received bytes: ");
    Serial.println(rx_total);
    Serial.print("  Time (ms): ");
    Serial.println(millisCountUp);

    return res == SUCCESS;
  }
  return 0;
}

bool send_command(byte device, const char *command) {
  Serial.print("!> ");
  Serial.println(command);
  
  return check_srq() &&
         check(cmd(MSG_UNLISTEN))        && check_srq() &&
         check(cmd(MSG_LISTEN | device)) && check_srq() &&
         check(tx_data(command))         && check_srq() &&
         check(cmd(MSG_UNLISTEN))        && check_srq() &&
         check(cmd(MSG_UNTALK))          && check_srq();
}

void setup() {
  DDRB = DDRC = DDRD = 0; // all inputs

  mode(TE_LISTEN, DATA_NO_EOI);

  PORTC |= PC_REN_MASK; // set REN false on GPIB

  // Uncomment these two lines to execute interface clear at controller initialisation
  //PORTC &= ~PC_IFC_MASK; // set IFC true on GPIB
  //delay(10);
  
  PORTC |= PC_IFC_MASK; // set IFC false on GPIB


  Serial.begin(115200);

  // set up 1ms interrupt
  TCCR1A = TCCR1C = 0;
  TCCR1B = (1 << CS10); //B101; // clkIO/1 prescaler
  OCR1A = 16000; // approx 1ms
  TIMSK1 = 0; //(1 << OCIE1A); // set output compare A match interrupt enable

  interrupts();


  //output_test(1);
  //input_test(1);

  Serial.println("\n\n\nController start");

  byte buf[RECEIVE_BUFFER_SIZE+1];

  if(send_query(MY_SCOPE, "*IDN?",    TEXT, buf, RECEIVE_BUFFER_SIZE)) Serial.println("OK");
  if(send_query(MY_SCOPE, "acquire?", TEXT, buf, RECEIVE_BUFFER_SIZE)) Serial.println("OK");

  /*
   * Select the waveform source(s) using the DATa:SOUrce command. If you want to transfer multiple waveforms, select more than one source.
2. Specify the waveform data format using DATa:ENCdg.
3. Specify the number of bytes per data point using DATa:WIDth.
4. Specify the portion of the waveform that you want to transfer using DATa:STARt and DATa:STOP.
5. Transfer waveform preamble information using WFMPRe? query.
6. Transfer waveform data from the digitizing oscilloscope using the CURVe?
query.
   */

  if(send_command(MY_SCOPE, "data:source ch1") 
  && send_command(MY_SCOPE, "data:encdg ascii")
  && send_command(MY_SCOPE, "data:width 2")
  && send_command(MY_SCOPE, "data:start 1")
  && send_command(MY_SCOPE, "data:stop 10")
  && send_query(MY_SCOPE, "wfmpre:ch1?", TEXT, buf, RECEIVE_BUFFER_SIZE)) {
    // 7 chars per sample, 500 samples will be 3.5K
    if(send_query(MY_SCOPE, "curve?", TEXT, buf, RECEIVE_BUFFER_SIZE)) {
      Serial.println("\n\n%%% Curve done");
    }
  }

  // Example text format hardcopy
  if(send_command(MY_SCOPE, "hardcopy:format epsmono") 
  && send_command(MY_SCOPE, "hardcopy:port gpib")
  && send_command(MY_SCOPE, "hardcopy:layout portrait")
  && send_query(MY_SCOPE, "hardcopy start", TEXT, buf, RECEIVE_BUFFER_SIZE)) {
      Serial.println("\n\n%%% Hardcopy done");
  }

  // Example binary format hardcopy. Convert base64 output with a suitable utility.
  if(send_command(MY_SCOPE, "hardcopy:format tiff") 
  && send_command(MY_SCOPE, "hardcopy:port gpib")
  && send_command(MY_SCOPE, "hardcopy:layout portrait")
  && send_query(MY_SCOPE, "hardcopy start", BINARY, buf, RECEIVE_BUFFER_SIZE)) {
      Serial.println("\n\n%%% Hardcopy done");
  }
}

void loop() {
}
