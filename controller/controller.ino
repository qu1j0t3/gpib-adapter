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

#include <ctype.h>

#include "controller.h"

const char *CTLR_VERSION = "2018-06-16 Controller by Toby Thain <toby@telegraphics.com.au>";

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
byte read_after_write = 1;
byte read_until_eoi = 1;
byte read_until_char;
byte command_eoi = 1;
byte eos = 3;
byte eot_enable = 0;
byte eot_char;
byte listen_only = 0;
unsigned read_timeout_10ms = 150;
byte status_byte = 0;

// nonstandard
byte verbose = 1;
byte binary_mode = 0;
byte cr_to_lf = 0;

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

volatile unsigned millisCountdown, millisCountUp;

byte transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return DIR_BUG;
  }

  PORTC |= PC_DAV_MASK; // data not valid
  
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
    
    if(!(mask & PB_ATN_FALSE_MASK)) Serial.print("  ATN");
    if(!(mask & PB_EOI_FALSE_MASK)) Serial.print("  EOI");
    Serial.println(mask & PB_PE_TOTEMPOLE ? "  PE(TP)" : "  ~PE(OC)");
#endif
    value = ~ value;
    PORTB = PB_TE_MASK | mask | (value >> 6);
    PORTD = value << 2;
    delayMicroseconds(2);
    
    // Wait until all acceptors are ready for data
    countdown(TRANSMIT_TIMEOUT_10MS);
    while(!(PINC & PC_NRFD_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NRFD was not raised within timeout. Aborting transmit");
      return TX_TIMEOUT;
    }
      
    PORTC &= ~PC_DAV_MASK; // data valid
    
    // Wait until all acceptors have accepted the data
    countdown(TRANSMIT_TIMEOUT_10MS);
    while(!(PINC & PC_NDAC_MASK) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NDAC was not raised within timeout. Aborting transmit");
      return TX_TIMEOUT;
    }
    
    PORTC |= PC_DAV_MASK; // data not valid
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

  if(millisCountdown == 0) {
    Serial.println("Data not valid within timeout. Aborting receive");
    return RX_TIMEOUT;
  }

  PORTC &= ~PC_NRFD_MASK; // not ready for data
  
  *value = ~((PINB << 6) | (PIND >> 2));
  byte res = PINB & PB_EOI_MASK ? SUCCESS : EOI; // remembering level -> logic inversion

  PORTC |= PC_NDAC_MASK; // accepted data
  
  return res;
}

byte tx_data(const char str[], bool use_eoi) {
  mode(TE_TALK, DATA_NO_EOI);
  for(const char *p = str; *p; ++p) {
    byte res = transmit(!use_eoi || p[1] ? DATA_NO_EOI : DATA_EOI, *p);
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
    if(res & IS_ERROR) {
      return res;
    } else if((res == EOI && read_until_eoi) || (!read_until_eoi && *p == read_until_char)) {
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
static byte buf[RECEIVE_BUFFER_SIZE+2];
  
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
      Serial.println("Direction not configured correctly for transmit/receive operation - probably a bug\n"); 
      return 0;
    case NO_LISTENERS:
      Serial.println("No listeners - Is adapter plugged into bus and devices are powered on?\n"); 
      return 0;
    case TX_TIMEOUT: Serial.println("Transmit timeout\n"); return 0;
    case RX_TIMEOUT: Serial.println("Receive timeout\n");  return 0;
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

bool device_listen(byte addr) {
  static char base64[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  
  if(check(cmd(MSG_TALK | addr))) {
    size_t n = 0, rx_total = 0, groups = 0;
    size_t sz = RECEIVE_BUFFER_SIZE;
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
        buf[n] = 0; // terminate buffer for caller convenience with short responses

        if(binary_mode) {
          buf[n] = buf[n+1] = buf[n+2] = 0; // zero out trailing octet group after defined data
          for(size_t i = 0; i < n; i += 3) {
            byte n1 = buf[i] >> 2;
            byte n2 = (buf[i] << 4) | (buf[i+1] >> 4);
            byte n3 = (buf[i+1] << 2) | (buf[i+2] >> 6);
            byte n4 = buf[i+2];
            Serial.write(base64[n1]);                             // Always defined
            Serial.write(base64[n2 & 0b111111]);                  // Always defined
            Serial.write(i >= n-1 ? '=' : base64[n3 & 0b111111]); // Only defined if we have data for p[1] and p[2]
            Serial.write(i >= n-2 ? '=' : base64[n4 & 0b111111]); // Only defined if we have data for p[2]
            ++groups;
            if(groups == 18) {
              Serial.write('\n');
              groups = 0;
            }
          }
        } else {
          if(cr_to_lf) {
            for(size_t i = 0; i < n; ++i) if(buf[i] == '\r') buf[i] = '\n';
          }
          Serial.write(buf, n);
        }
      } else {
        return 0;
      }
    } while(res == BUFFER_FULL);
    if(binary_mode) Serial.println("\n%%% Base64 end");

    if(rx_total > sz) { // Only show stats on "long" responses
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
  Serial.print("!> ");
  Serial.println(command);
  
  return check(cmd(MSG_UNLISTEN)) &&
         check(cmd(MSG_LISTEN | device)) &&
         check(tx_data(command, command_eoi)) &&
         check(cmd(MSG_UNLISTEN)) &&
         check(cmd(MSG_UNTALK));
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
  TCCR1B = (1 << CS11); // clkIO/8 prescaler
  OCR1A = 20000; // approx 10ms
  TIMSK1 = 0; //(1 << OCIE1A); // set output compare A match interrupt enable

  interrupts();


  //output_test(1);
  //input_test(1);

  Serial.println("\n\n\nController start");
/*
  if(send_query(MY_SCOPE, "*IDN?",    TEXT, buf, RECEIVE_BUFFER_SIZE)) Serial.println("OK");
  if(send_query(MY_SCOPE, "acquire?", TEXT, buf, RECEIVE_BUFFER_SIZE)) Serial.println("OK");

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
  }*/
}

void prologix_setting(byte *p, byte num_params, byte param_value) {
  if(num_params) {
    *p = param_value;
  } else {
    Serial.println(*p);
  }
}

void loop() {
  for(;;) {
    size_t n;
    bool controller_command = 0;

    /*
    prompt = check_srq();
    if(read_sesr) {
      read_sesr = 0;
      send_query(device, (char*)"*ESR?", TEXT, buf, RECEIVE_BUFFER_SIZE);
      byte sesr = atoi((char*)buf);
      if(sesr) {
        Serial.print("\nEvent Status Register: ");
        if(sesr & 0b10000000) Serial.print(" PowerOn");
        if(sesr & 0b01000000) Serial.print(" UserRequest");
        if(sesr & 0b00100000) Serial.print(" CommandError");
        if(sesr & 0b00010000) Serial.print(" ExecutionError");
        if(sesr & 0b00001000) Serial.print(" DeviceError");
        if(sesr & 0b00000100) Serial.print(" QueryError");
        if(sesr & 0b00000010) Serial.print(" RequestControl");
        if(sesr & 0b00000001) Serial.print(" OperationComplete");
        Serial.write('\n');
        send_query(device, (char*)"EVMSG?", TEXT, buf, RECEIVE_BUFFER_SIZE);
      }
    }*/

    bool escape_next = 0;
    for(n = 0; n < RECEIVE_BUFFER_SIZE;) {
      int c = Serial.read();
      if(c != -1) {
        if(!escape_next && c == ESC) {
          escape_next = 1;
        } else if(!escape_next && (c == '\n' || c == '\r')) {
          break;
        } else if(escape_next || (c != '\n' && c != '\r' && c != '+' && c != ESC)) {
          buf[n++] = c;
          escape_next = 0;
        } else if(n == 0 && c == '+') { // unescaped + at beginning of input starts a controller command
          buf[n++] = c;
          controller_command = 1;
        } else if(controller_command) {
          if(n == 1 && c == '+') { // the 2nd + that is part of a controller command
            buf[n++] = c;
          } else {
            controller_command = 0; // this is really a kind of syntax error
            n = 0; // throw away the partial command
          }
        }
      }
    }
    
    if(controller_command) {
      char *command = (char*)buf + 2;
      byte num_params = 0;
      int param_values[3];
      char *param_strings[3];
      byte i;
      // Find end of command word
      for(i = 2; i < n && !isspace(buf[i]); ++i)
        ;
      buf[i++] = buf[n] = 0; // terminate command word, and entire command
      
      while(num_params < 3 && i < n) {
        while(buf[i] && isspace(buf[i])) ++i;
        param_strings[num_params] = buf+i;
        param_values[num_params++] = atoi((char*)(buf+i));
        while(buf[i] && !isspace(buf[i])) ++i;
        buf[i] = 0; // terminate parameter string
      }

      if(!strcmp(command, "addr")) {
        if(num_params == 0) {
          Serial.println(addr);
          if(sec_addr) Serial.println(sec_addr);
        } else if(num_params == 1 && param_values[0] >= 0 && param_values[0] <= 30) {
          addr = param_values[0];
          sec_addr = 0;
        } else if(param_values[1] >= 96 && param_values[1] <= 126) {
          addr = param_values[0];
          sec_addr = param_values[1];
        }
      } else if(!strcmp(command, "auto")) {
        prologix_setting(&read_after_write, num_params, param_values[0]);
        
        if(num_params && addr != NO_DEVICE) {
          if(param_values[0]) { // address device to TALK
            cmd(MSG_TALK | addr);
          } else { // address device to LISTEN
            cmd(MSG_LISTEN | addr);
          }
        }
      } else if(!strcmp(command, "clr")) {
        if(addr != NO_DEVICE) {
          /*TODO*/
        }
      } else if(!strcmp(command, "eoi")) {
        prologix_setting(&command_eoi, num_params, param_values[0]);
      } else if(!strcmp(command, "eos")) {
        prologix_setting(&eos, num_params, param_values[0]);
      } else if(!strcmp(command, "eot_enable")) {
        prologix_setting(&eot_enable, num_params, param_values[0]);
      } else if(!strcmp(command, "eot_char")) {
        prologix_setting(&eot_char, num_params, param_values[0]);
      } else if(!strcmp(command, "ifc")) {
        PORTC &= ~PC_IFC_MASK; // set IFC true on GPIB
        delayMicroseconds(150);
        PORTC |= PC_IFC_MASK; // set IFC false on GPIB
      } else if(!strcmp(command, "llo")) {
        /*TODO*/
      } else if(!strcmp(command, "lon")) {
        prologix_setting(&listen_only, num_params, param_values[0]);
      } else if(!strcmp(command, "mode")) {
        Serial.println("Only controller mode is currently supported");
      } else if(!strcmp(command, "read")) {
        if(num_params == 0) {
          read_until_eoi = 1;
        } else {
          read_until_eoi = !strcmp(param_strings[0], "eoi");
          read_until_char = param_values[0];
        }
        if(addr != NO_DEVICE) {
          device_listen(addr);
        }
      } else if(!strcmp(command, "read_tmo_ms")) {
        if(num_params == 0) {
          Serial.println(read_timeout_10ms * 10);
        } else {
          read_timeout_10ms = param_values[0]/10;
        }
      } else if(!strcmp(command, "rst")) {
        /*TODO - probably use watchdog timer*/
      } else if(!strcmp(command, "savecfg")) {
        /*not impl*/
      } else if(!strcmp(command, "spoll")) {
        serialpoll(num_params ? param_values[0] : addr);
      } else if(!strcmp(command, "srq")) {
        Serial.println(!(PINC & PC_SRQ_MASK));
      } else if(!strcmp(command, "status")) {
        prologix_setting(&status_byte, num_params, param_values[0]);
      } else if(!strcmp(command, "trg")) {
        /*TODO*/
      } else if(!strcmp(command, "ver")) {
        Serial.println(CTLR_VERSION);
      } else if(!strcmp(command, "help")) {
        /*TODO*/
      } else if(!strcmp(command, "b64")) {
        prologix_setting(&binary_mode, num_params, param_values[0]);
      } else if(!strcmp(command, "cr2lf")) {
        prologix_setting(&cr_to_lf, num_params, param_values[0]);
      } else if(!strcmp(command, "verbose")) {
        prologix_setting(&verbose, num_params, param_values[0]);
      }
    } else {
      if(!(eos & 0b10)) buf[n++] = '\r';
      if(!(eos & 0b01)) buf[n++] = '\n';
      buf[n] = 0;
      
      if(addr != NO_DEVICE) {
        send_command(addr, (char*)buf);
        if(read_after_write) {
          device_listen(addr);
        }
      }
    }
    
  }
}
