/*
    This file is part of "GPIB Adapter", an Arduino based controller for GPIB (IEEE 488).
    Copyright (C) 2018-2020 Toby Thain, toby@telegraphics.com.au

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

#include <ctype.h>
#include <avr/wdt.h>
#include <stdint.h>

//#define DEBUG

typedef uint8_t byte;

#include "controller.h"

const char *CTLR_VERSION = "2020-08-29 GPIB/Arduino Controller by Toby Thain <toby@telegraphics.com.au>";

#define PB_OUTPUTS    0b110100 // PORTB Pins that are always outputs
#define PB_BIDI_FWD   0b001011 // PORTB Bidirectional pins that follow TALK ENABLE (transmit when TE is high)
#define PC_OUTPUTS    0b110000 // PORTC Pins that are always outputs
#define PC_BIDI_FWD   0b000010 // PORTC Bidirectional pins that follow TALK ENABLE
#define PD_BIDI_FWD 0b11111100 // PORTD Bidirectional pins that follow TALK ENABLE

#define ESC 033

#define TRANSMIT_TIMEOUT_10MS 50

// Prologix protocol state

byte addr = NO_DEVICE;
byte sec_addr = 0;
byte read_after_write = 0;
byte read_until_eoi;
byte read_until_char;
byte read_until_char_set;
byte command_eoi = 1;
byte eos = 3;
byte eot_enable = 0;
byte eot_char;
byte listen_only = 0;
unsigned read_timeout_10ms = 300;
size_t bytes_sent;
byte status_byte = 0;

// nonstandard
byte verbose = 0;
byte binary_mode = 0;
byte cr_to_lf = 0;

byte num_params;
byte param_values[30];
      
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

    // EOI follows direction of Talk Enable, UNLESS ATN is LOW, when it is the opposite of Talk Enable.
    if(!(atn_eoi_pe & PB_ATN_MASK)) {
      PORTB |= PB_EOI_MASK;
      DDRB  |= PB_EOI_MASK; // EOI is output
    }
  }
}

volatile unsigned millisCountdown, millisCountUp;

byte transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return DIR_BUG;
  }

  PORTC |= PC_DAV_MASK; // data not valid
  
  // Acceptors will initialise NRFD to low (True - none are ready for data),
  // and set NDAC to low (True - none have accepted the data)

  if((PINC & (PC_NRFD_MASK | PC_NDAC_MASK)) == (PC_NRFD_MASK | PC_NDAC_MASK)) {
    return NO_LISTENERS;
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

    if(!(mask & PB_ATN_MASK)) Serial.print("  ATN");
    if(!(mask & PB_EOI_MASK)) Serial.print("  EOI");
    Serial.println(mask & PB_PE_TOTEMPOLE ? "  PE(TP)" : "  ~PE(OC)");
#endif
    value = ~ value;
    PORTB = PB_TE_MASK | mask | (value >> 6);
    PORTD = value << 2;
    // "All high rate Talkers should use a minimum multiline message settling time
    // (T1 in the IEEE-488 Standard) of 350ns."
    delayMicroseconds(2);

    // Wait until all acceptors are ready for data (NRFD high)
    countdown(TRANSMIT_TIMEOUT_10MS);
    while(!(PINC & PC_NRFD_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) { // NRFD was not raised within timeout
      return TX_NRFD_TIMEOUT;
    }

    PORTC &= ~PC_DAV_MASK; // data valid

    // Wait until all acceptors have accepted the data (NDAC high)
    countdown(TRANSMIT_TIMEOUT_10MS);
    while(!(PINC & PC_NDAC_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) { // NDAC was not raised within timeout
      return TX_NDAC_TIMEOUT;
    }

    PORTC |= PC_DAV_MASK; // data not valid (DAV high)
    // Indicates to all acceptors that data on the DIO lines 
    // must now be considered not valid.

    // One byte has been transferred
    return SUCCESS;
  }
}

byte receive(byte *value){
  if(PORTB & PB_TE_MASK) { // interface must be in Listen direction
    return DIR_BUG;
  }

  PORTC &= ~PC_NDAC_MASK; // not accepted data
  PORTC |= PC_NRFD_MASK;  // ready for data

  // wait for data valid to go low (true)
  countdown(read_timeout_10ms);
  while((PINC & PC_DAV_MASK) && millisCountdown)
    ;

  if(millisCountdown == 0) { // Data not valid within timeout
    return RX_TIMEOUT;
  }

  PORTC &= ~PC_NRFD_MASK; // not ready for data

  byte v = *value = ~((PINB << 6) | (PIND >> 2));
  byte eoi = PINB & PB_EOI_MASK;
  byte res = eoi ? SUCCESS : EOI; // remembering level -> logic inversion

  PORTC |= PC_NDAC_MASK; // accepted data

#ifdef DEBUG
    char b[9];
    eight_bits(v, b);

    Serial.print("received: ");
    Serial.print(b);
    b[0] = b[2] = ' ';
    b[1] = v >= ' ' && v < 0x7f ? v : ' ';
    b[3] = 0;
    Serial.print(b);
    if(!eoi) Serial.print("  EOI");
    Serial.print('\r');
    Serial.print('\n');
#endif
  return res;
}

byte tx_data(byte str[], bool use_eoi, size_t n) {
  mode(TE_TALK, DATA_NO_EOI);
  for(size_t i = 0; i < n; ++i) {
    byte res = transmit(i == n-1 && use_eoi ? DATA_EOI : DATA_NO_EOI, str[i]);
    if(res != SUCCESS) {
      bytes_sent = i;
      return res;
    }
  }
  return SUCCESS;
}

byte rx_data(byte *buf, size_t max, size_t *count) {
  // This might be called in a block loop, so don't bother
  // changing direction unless necessary.
  if(PORTB & PB_TE_MASK) mode(TE_LISTEN, DATA_NO_EOI);

  size_t n = 0;
  for(byte *p = buf; n < max; ++n, ++p) {
    byte res = receive(p);
    if(res & IS_ERROR) {
      return res;
    } else if((read_until_eoi && res == EOI) || (read_until_char_set && *p == read_until_char)) {
      if(res == EOI && eot_enable) {
        p[1] = eot_char;
        ++n;
      }
      *count = ++n;
      return SUCCESS;
    }
  }
  *count = n;
  return BUFFER_FULL;
}

#define RECEIVE_BUFFER_SIZE 0x100
static byte main_buffer[RECEIVE_BUFFER_SIZE+3];

byte cmd(byte msg) {
  mode(TE_TALK, CMD_NO_EOI);
#ifdef DEBUG
  Serial.print("cmd:  ");
#endif
  return transmit(CMD_NO_EOI, msg);
}

bool read_sesr = 0;
bool message_available = 0;

void serialpoll(byte addr) {
  byte stat;

  if(cmd(MSG_UNLISTEN) == SUCCESS &&
     cmd(MSG_SER_POLL_ENB) == SUCCESS &&
     cmd(MSG_TALK | addr) == SUCCESS &&
     (mode(TE_LISTEN, DATA_NO_EOI), receive(&stat)) == SUCCESS &&
     cmd(MSG_SER_POLL_DIS) == SUCCESS)
  {
    Serial.print("\nSRQ Status: ");
    if(stat & 0b1000000) Serial.print(" RequestService");
    if(stat & 0b0100000) {
      Serial.print(" EventStatusBit");
      read_sesr |= 1;
    }
    if(stat & 0b0010000) {
      Serial.print(" MessageAvailable");
      message_available |= 1;
    }
    Serial.println();
  }
}

bool check_srq() {
  if(!(PINC & PC_SRQ_MASK)) {
    serialpoll(addr);
    return 1;
  }
  return 0;
}

bool check(byte res) {
  switch(res) {
    case EOI:
    case SUCCESS:
      return 1;
    case DIR_BUG:
      Serial.println("Dir not configured correctly for tx/rx - bug?");
      return 0;
    case NO_LISTENERS:
      if(verbose) Serial.println("No listeners - Adapter plugged in and devices on?");
      return 0;
    case TX_NDAC_TIMEOUT:
    case TX_NRFD_TIMEOUT:
      if(verbose) {
        Serial.print("Transmit timeout after ");
        Serial.print(bytes_sent);
        Serial.print(" bytes; ");
        Serial.println(res == TX_NDAC_TIMEOUT ? "data not accepted" : "acceptors not ready");
      }
      return 0;
    case RX_TIMEOUT:
      if(verbose) Serial.println("Receive timeout");
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

byte base64_groups;

void send_base64(byte *buf, size_t n) {
  static char base64[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for(size_t i = 0; i < n; i += 3) {
    byte b1 = (i+1) < n ? buf[i+1] : 0;
    byte b2 = (i+2) < n ? buf[i+2] : 0;
    Serial.write(base64[buf[i] >> 2]);                                          // Always defined
    Serial.write(base64[((buf[i] << 4) | (b1 >> 4)) & 0b111111]);               // Always defined
    Serial.write((i+1) < n ? base64[((b1 << 2) | (b2 >> 6)) & 0b111111] : '='); // Only defined if we have data for p[1] and p[2]
    Serial.write((i+2) < n ? base64[b2 & 0b111111] : '=');                      // Only defined if we have data for p[2]
    ++base64_groups;
    if(base64_groups == 18) {
      Serial.write('\r');
      Serial.write('\n');
      base64_groups = 0;
    }
  }
}

bool device_listen(byte addr, bool binary_mode) {
  if(check(cmd(MSG_TALK | addr))) {
    size_t n = 0, rx_total = 0;
    size_t sz = RECEIVE_BUFFER_SIZE;
    byte res;

    millisCountUp = 0;

    if(binary_mode) {
      sz -= (sz % 3) + 3; // make buffer a multiple of 3 for correct Base64 encoding
      base64_groups = 0;
    }

    do {
      res = rx_data(main_buffer, sz, &n);
      if(res == SUCCESS || res == BUFFER_FULL) {
        if(verbose && binary_mode && !rx_total) {
          Serial.println("%%% Base64 data:");
        }
        rx_total += n;
        main_buffer[n] = 0; // terminate buffer for caller convenience with short responses

        if(binary_mode) {
          send_base64(main_buffer, n);
        } else {
          if(cr_to_lf) {
            for(size_t i = 0; i < n; ++i) if(main_buffer[i] == '\r') main_buffer[i] = '\n';
          }
          Serial.write(main_buffer, n);
        }
      } else {
        check(res);
        return 0;
      }
    } while(res == BUFFER_FULL);

    // The end marker is very helpful to hosts receiving binary (base64) data, so always print it.
    if(/*verbose &&*/ binary_mode) Serial.println("\n%%% Base64 end\n");

    if(verbose && rx_total > sz) { // Only show stats on "long" responses
      Serial.print("  Received bytes: ");
      Serial.println(rx_total);
      Serial.print("  Time (ms): ");
      Serial.println(millisCountUp);
    }
    Serial.flush();

    return res == SUCCESS;
  }
  return 0;
}

bool send_command(byte device, const char *command) {
  if(device == NO_DEVICE){
    if(verbose) Serial.println("Device address not set. Use ++addr <address>");
    return 0;
  }

  return check(cmd(MSG_UNLISTEN)) &&
         check(cmd(MSG_LISTEN | device)) &&
         check(tx_data((byte*)command, command_eoi, strlen(command))) &&
         check(cmd(MSG_UNLISTEN)) &&
         check(cmd(MSG_UNTALK));
}

void prologix_setting(byte *p) {
  if(num_params) {
    *p = param_values[0];
  } else {
    Serial.println(*p);
  }
}

void setup() {
  DDRB = DDRC = DDRD = 0; // all inputs

  //MCUSR = 0;
  //wdt_disable();

  mode(TE_LISTEN, DATA_NO_EOI);

  PORTC |= PC_REN_MASK; // set REN false on GPIB

  // Uncomment these two lines to execute interface clear at controller initialisation
  //PORTC &= ~PC_IFC_MASK; // set IFC true on GPIB
  //delay(10);

  PORTC |= PC_IFC_MASK; // set IFC false on GPIB

  // set up 10ms interrupt
  TCCR1A = TCCR1C = 0;
  TCCR1B = (1 << CS11); // clkIO/8 prescaler
  OCR1A = 20000; // approx 10ms
  TIMSK1 = 0; //(1 << OCIE1A); // set output compare A match interrupt enable

  interrupts();

  Serial.begin(115200);

  Serial.print("\r\n\r\n");
  Serial.println(CTLR_VERSION);

  Serial.println("++v 1  for interactive session. See  ++help. Ready.");
/*
  if(send_query(MY_SCOPE, "*IDN?",    TEXT, main_buffer, RECEIVE_BUFFER_SIZE)) Serial.println("OK");
  if(send_query(MY_SCOPE, "acquire?", TEXT, main_buffer, RECEIVE_BUFFER_SIZE)) Serial.println("OK");

  if(send_command(MY_SCOPE, "data:source ch1")
  && send_command(MY_SCOPE, "data:encdg ascii")
  && send_command(MY_SCOPE, "data:width 2")
  && send_command(MY_SCOPE, "data:start 1")
  && send_command(MY_SCOPE, "data:stop 500")
  && send_query(MY_SCOPE, "wfmpre:ch1?", TEXT, main_buffer, RECEIVE_BUFFER_SIZE)) {
    // 7 chars per sample, 500 samples will be 3.5K
    if(send_query(MY_SCOPE, "curve?", TEXT, main_buffer, RECEIVE_BUFFER_SIZE)) {
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
  && send_query(MY_SCOPE, "hardcopy start", BINARY, main_buffer, RECEIVE_BUFFER_SIZE)) {
      Serial.println("\n\n%%% Hardcopy done");
  }*/
}

void loop() {
  for(;;) {
    size_t n;
    bool escape_next;
    bool controller_command;
    
    if(addr != NO_DEVICE && verbose) {
      check_srq();
      if(read_sesr) {
        read_sesr = 0;
        send_command(addr, (char*)"*ESR?");
        device_listen(addr, 0);
        byte sesr = atoi((char*)main_buffer);
        if(sesr && verbose) {
          Serial.print("\nEvent Status Register: ");
          if(sesr & 0b10000000) Serial.print(" PowerOn");
          if(sesr & 0b01000000) Serial.print(" UserRequest");
          if(sesr & 0b00100000) Serial.print(" CommandError");
          if(sesr & 0b00010000) Serial.print(" ExecutionError");
          if(sesr & 0b00001000) Serial.print(" DeviceError");
          if(sesr & 0b00000100) Serial.print(" QueryError");
          if(sesr & 0b00000010) Serial.print(" RequestControl");
          if(sesr & 0b00000001) Serial.print(" OperationComplete");
          Serial.write('\r');
          Serial.write('\n');
          send_command(addr, "EVMSG?");
          device_listen(addr, 0);
        }
      }
    }
    
    if(verbose) {
      Serial.print("\r\n> ");
    }
      
    for(n = 0, controller_command = 0, escape_next = 0; n < RECEIVE_BUFFER_SIZE;) {
      int c = Serial.read();
      if(c != -1) {
        main_buffer[n] = c;
        if((c == '\n' || c == '\r') && !escape_next) {
          if(verbose) {
            Serial.write('\r');
            Serial.write('\n');
          }
          break;
        } else if(c == ESC && !escape_next) {
          escape_next = 1;
        } else if((c != '\n' && c != '\r' && c != '+' && c != ESC) || escape_next) {
          ++n;
          escape_next = 0;
          if(verbose) Serial.write(c);
        } else if(n == 0 && c == '+') { // unescaped + at beginning of input starts a controller command
          ++n;
          controller_command = 1;
          if(verbose) Serial.write(c);
        } else if(controller_command) {
          if(n == 1 && c == '+') { // the 2nd + that is part of a controller command
            ++n;
            if(verbose) Serial.write(c);
          } else {
            controller_command = 0; // syntax error
            n = 0; // throw away the partial command
          }
        }
      }
    }
    
    if(controller_command) {
      char *command = (char*)main_buffer + 2;
      char *first_param = NULL;
      byte i;

      // Find end of command word
      for(i = 2; i < n && !isspace(main_buffer[i]); ++i)
        ;
      main_buffer[i++] = main_buffer[n] = 0; // terminate command word, and entire command

      // collect up to three parameters separated by whitespace
      // set `first_param` to point to the first parameter string (nul terminated)
      for(num_params = 0; num_params < 3 && i < n;) {
        while(main_buffer[i] && isspace(main_buffer[i]))
            ++i;
        if(num_params == 0)
            first_param = (char*)main_buffer + i;
        param_values[num_params++] = atoi((char*)main_buffer + i);
        while(main_buffer[i] && !isspace(main_buffer[i]))
            ++i;
        main_buffer[i] = 0; // terminate parameter string
      }

      if(!strcmp(command, "addr")) {
        if(num_params == 0) {
          Serial.println(addr);
          if(sec_addr) Serial.println(sec_addr);
        } else if(num_params == 1 && param_values[0] <= 30) {
          addr = param_values[0];
          sec_addr = 0;
        } else if(param_values[1] >= 96 && param_values[1] <= 126) {
          addr = param_values[0];
          sec_addr = param_values[1];
        }
      } else if(!strcmp(command, "auto")) {
        prologix_setting(&read_after_write);
        
        if(read_after_write == 2) {
          read_until_eoi = 1;
          read_until_char_set = 0;
        } else if(read_after_write > 2) {
          read_until_eoi = 0;
          read_until_char = read_after_write;
          read_until_char_set = 1;
        }

        if(num_params && addr != NO_DEVICE) {
          if(param_values[0]) { // address device to TALK
            cmd(MSG_TALK | addr);
          } else { // address device to LISTEN
            cmd(MSG_LISTEN | addr);
          }
        }
      } else if(!strcmp(command, "clr")) {
        static char sel_dev_clr[] = {MSG_SEL_DEV_CLR, 0};
        send_command(addr, sel_dev_clr);
      } else if(!strcmp(command, "eoi")) {
        prologix_setting(&command_eoi);
      } else if(!strcmp(command, "eos")) {
        prologix_setting(&eos);
      } else if(!strcmp(command, "eot_enable")) {
        prologix_setting(&eot_enable);
      } else if(!strcmp(command, "eot_char")) {
        prologix_setting(&eot_char);
      } else if(!strcmp(command, "ifc")) {
        PORTC &= ~PC_IFC_MASK; // set IFC true on GPIB
        delayMicroseconds(150);
        PORTC |= PC_IFC_MASK; // set IFC false on GPIB
      } else if(!strcmp(command, "llo")) {
        static char local_lockout[] = {MSG_LOCALLOCKOUT, 0};
        send_command(addr, local_lockout);
      } else if(!strcmp(command, "loc")) {
        static char go_to_local[] = {MSG_GO_TO_LOCAL, 0};
        send_command(addr, go_to_local);
      } else if(!strcmp(command, "lon")) {
        prologix_setting(&listen_only);
      } else if(!strcmp(command, "mode")) {
        Serial.println("Only controller mode is supported");
      } else if(!strcmp(command, "read")) {
        if(!strcmp(first_param, "eoi")) {
          read_until_eoi = 1;
          read_until_char_set = 0;
        } else {
          read_until_eoi = 0;
          read_until_char_set = num_params > 0;
          read_until_char = param_values[0];
        }
        if(addr != NO_DEVICE) {
          device_listen(addr, binary_mode);
        } else {
          Serial.println("Device address not set. Use ++addr <address>");
        }
      } else if(!strcmp(command, "read_tmo_ms")) {
        if(num_params == 0) {
          Serial.println(read_timeout_10ms * 10);
        } else {
          read_timeout_10ms = atol(first_param)/10;
        }
      } /*else if(!strcmp(command, "rst")) {
        // This doesn't work yet; device goes into a reset loop if used
        //WDTCSR = (1 << WDCE) | (1 << WDE);
        wdt_enable(WDTO_15MS);
        while(1) ;
      }*/ /*else if(!strcmp(command, "savecfg")) {
        not impl
      }*/ else if(!strcmp(command, "spoll")) {
        serialpoll(num_params ? param_values[0] : addr);
      } else if(!strcmp(command, "srq")) {
        Serial.println(!(PINC & PC_SRQ_MASK));
      } else if(!strcmp(command, "status")) {
        prologix_setting(&status_byte);
      } else if(!strcmp(command, "trg")) {
        static char group_exec_trigger[] = {MSG_GRP_EXEC_TRG, 0};
        if(num_params == 0) {
          send_command(addr, group_exec_trigger);
        } else {
          if(check(cmd(MSG_UNLISTEN))) {
            byte j;
            for(j = 0; j < num_params; ++j) {
              if(param_values[j] <= 30 && !check(cmd(MSG_LISTEN | param_values[j]))) {
                break;
              }
            }
            if(j == num_params) {
              check(cmd(MSG_GRP_EXEC_TRG));
            }
          }
        }
      } else if(!strcmp(command, "ver")) {
        Serial.println(CTLR_VERSION);
      } else if(!strcmp(command, "help")) {

        Serial.println("Supported Prologix commands:");
        Serial.println("  addr [<pad> [<sad>]]");
        Serial.println("  auto [0|1|2(eoi)|char]");
        Serial.println("  clr");
        Serial.println("  eoi [0|1]");
        Serial.println("  eos [0|1|2|3]");
        Serial.println("  eot_enable [0|1]");
        Serial.println("  eot_char [<char>]");
        Serial.println("  ifc");
        Serial.println("  llo");
        Serial.println("  loc");
        //Serial.println("lon")) {
        //Serial.println("mode")) {
        Serial.println("  read [eoi|<char>]");
        Serial.println("  read_tmo_ms [<time>]");
        //Serial.println("  rst");
        //Serial.println("savecfg")) {
        Serial.println("  spoll [<pad> [<sad>]]");
        Serial.println("  srq");
        //Serial.println("status")) {
        Serial.println("  trg [<pad1> [<sad1>] ...]");
        Serial.println("  ver");
        Serial.println("  help");
        Serial.println("Non-standard:");
        Serial.println("  b64 [0|1] - base64 format for response");
        Serial.println("  cr2lf [0|1] - map CR to LF in response");
        Serial.println("  v[erbose] [0|1|2] - 1 = interactive mode; sets auto=1. 2 = report status after message");

      } else if(!strcmp(command, "b64")) {
        prologix_setting(&binary_mode);
      } else if(!strcmp(command, "cr2lf")) {
        prologix_setting(&cr_to_lf);
      } else if(!strcmp(command, "verbose") || !strcmp(command, "v")) {
        prologix_setting(&verbose);
        
        if (verbose) {
          read_after_write = 1;
          
          Serial.print("Receive timeout: ");
          Serial.print(read_timeout_10ms);
          Serial.print("0ms  Transmit timeout: ");
          Serial.print(TRANSMIT_TIMEOUT_10MS);
          Serial.println("0ms");
        }
      } else if(verbose) {
        Serial.print("Unknown command: ");
        Serial.println(command);
      }
    } else if(n) {
      if(!(eos & 0b10)) main_buffer[n++] = '\r';
      if(!(eos & 0b01)) main_buffer[n++] = '\n';
      main_buffer[n] = 0;

      byte success = send_command(addr, (char*)main_buffer);
      if(success && read_after_write) { // FIXME - will stop at a NUL byte
        device_listen(addr, binary_mode);
      }
    }
  }
}
