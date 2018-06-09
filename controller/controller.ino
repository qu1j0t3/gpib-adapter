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


// Address of instrument. In my case, Tektronix TDS460A.
// Check GPIB address on the Shift-STATUS screen, I/O tab.

#define MY_SCOPE 2


void mode(bool talk, bool atn){
  // SAFETY: Arduino outputs connected to transceiver lines configured as outputs are
  //         likely to violate driver current limits. Set all Arduino pins to high-Z,
  //         so that none are outputs when transceiver lines are switched to outputs by
  //         the change in Talk Enable line.
  //           As high-Z pins, these will be pulled up or down according to
  //         the wired resistors on the terminal side (GPIO pin). 
  //         All are pulled up to Vcc except NRFD and NDAC which are pulled down to GND.
          
  pinMode(D1_PIN,   INPUT); // TODO: The high level routines are very inefficient,
  pinMode(D2_PIN,   INPUT); //       could change them to direct port operations, but our
  pinMode(D3_PIN,   INPUT); //       serial speeds probably make this fairly unimportant.
  pinMode(D4_PIN,   INPUT);
  pinMode(D5_PIN,   INPUT);
  pinMode(D6_PIN,   INPUT);
  pinMode(D7_PIN,   INPUT);
  pinMode(D8_PIN,   INPUT);
  pinMode(EOI_PIN,  INPUT);
  pinMode(DAV_PIN,  INPUT);
  pinMode(NRFD_PIN, INPUT);
  pinMode(NDAC_PIN, INPUT);

  // Set outputs to match the external pullups/pulldowns,
  // to prevent glitches when pin is changed from high-Z to output.
  
  if(talk) {
    // these pins are becoming outputs
    digitalWrite(D1_PIN,   HIGH);
    digitalWrite(D2_PIN,   HIGH);
    digitalWrite(D3_PIN,   HIGH);
    digitalWrite(D4_PIN,   HIGH);
    digitalWrite(D5_PIN,   HIGH);
    digitalWrite(D6_PIN,   HIGH);
    digitalWrite(D7_PIN,   HIGH);
    digitalWrite(D8_PIN,   HIGH);
    digitalWrite(EOI_PIN,  HIGH);
    digitalWrite(DAV_PIN,  HIGH); // data not valid
  } else {
    // these pins are becoming outputs
    if(!atn) digitalWrite(EOI_PIN,  HIGH);
    digitalWrite(NRFD_PIN, LOW); // not ready for data
    digitalWrite(NDAC_PIN, LOW); // not accepted data
  }
  
  digitalWrite(TE_PIN, talk); // Talk Enable
  
  byte fwd = talk ? OUTPUT : INPUT;
  byte rev = talk ? INPUT  : OUTPUT;
  
  pinMode(D1_PIN,   fwd);
  pinMode(D2_PIN,   fwd);
  pinMode(D3_PIN,   fwd);
  pinMode(D4_PIN,   fwd);
  pinMode(D5_PIN,   fwd);
  pinMode(D6_PIN,   fwd);
  pinMode(D7_PIN,   fwd);
  pinMode(D8_PIN,   fwd);
  pinMode(EOI_PIN,  atn == HIGH ? fwd : OUTPUT /*assuming DC is Low*/);
  pinMode(DAV_PIN,  fwd);
  pinMode(NRFD_PIN, rev);
  pinMode(NDAC_PIN, rev);

  digitalWrite(ATN_PIN, atn);
}

#define TRANSMIT_TIMEOUT_MS 500

volatile unsigned millisCountdown = 0;

byte transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return ERR;
  } /*else if(!(PINC & PC_SRQ_MASK)) {
    return SRQ;
  }*/
  
  digitalWrite(DAV_PIN, HIGH); // data not valid
  
  if(digitalRead(NRFD_PIN) && digitalRead(NDAC_PIN)) {
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
    while(!digitalRead(NRFD_PIN) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NRFD was not raised within 500ms. Aborting transmit");
      return TIMEOUT;
    }
      
    digitalWrite(DAV_PIN, LOW); // data valid
    
    // Wait until all acceptors have accepted the data
    countdown(TRANSMIT_TIMEOUT_MS);
    while(!digitalRead(NDAC_PIN) && millisCountdown)
      ;

    if(millisCountdown == 0) {
      Serial.println("NDAC was not raised within 500ms. Aborting transmit");
      return TIMEOUT;
    }
    
    digitalWrite(DAV_PIN, HIGH); // data not valid
    return SUCCESS;
  }
}

byte receive(byte *value){
  if(PORTB & PB_TE_MASK) { // interface must be in Listen direction
    return ERR;
  } /*else if(!(PINC & PC_SRQ_MASK)) {
    return SRQ;
  }*/
  
  digitalWrite(NDAC_PIN, LOW);  // not accepted data
  digitalWrite(NRFD_PIN, HIGH); // ready for data

  // wait for data valid to go low (true)
  countdown(5000);
  while(digitalRead(DAV_PIN) && millisCountdown)
    ;

  if(millisCountdown == 0) {
    Serial.println("Data not valid within 5 sec. Aborting receive");
    return TIMEOUT;
  }
  
  digitalWrite(NRFD_PIN, LOW);  // not ready for data
  
  *value = ~((PINB << 6) | (PIND >> 2));
  bool eoi_level = digitalRead(EOI_PIN);
  
  digitalWrite(NDAC_PIN, HIGH); // accepted data
  
  return eoi_level ? SUCCESS : EOI; // remembering level -> logic inversion
}

byte tx_data(const char str[]) {
  mode(TE_TALK, /*ATN*/GPIB_FALSE);
  for(const char *p = str; *p; ++p) {
    byte res = transmit(p[1] ? DATA_NO_EOI : DATA_EOI, *p);
    if(res != SUCCESS) {
      return res;
    }
  }
  return SUCCESS;
}

byte rx_data(byte *buf, size_t max, size_t *count) {
  // This might be called in a block loop, so don't bother
  // changing direction unless necessary.
  if(PORTB & PB_TE_MASK) mode(TE_LISTEN, /*ATN*/GPIB_FALSE);
  
  size_t n = 0;
  for(byte *p = buf; n < max; ++n, ++p) {
    byte res = receive(p);
    if(res == EOI || *p == '\n') { // FIXME: What about binary data
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
  mode(TE_TALK, /*ATN*/GPIB_TRUE);
#ifdef DEBUG
  Serial.print("cmd:  ");
#endif
  return transmit(PB_PE_TOTEMPOLE | PB_EOI_FALSE_MASK, msg);
}

void serialpoll() {
  byte stat;
  char bits[9];
  
  Serial.println("Serial poll");
  cmd(MSG_UNLISTEN);
  cmd(MSG_SER_POLL_ENB);
  cmd(MSG_TALK | MY_SCOPE);
  mode(TE_LISTEN, /*ATN*/GPIB_FALSE);
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
}

void countdown(unsigned ms) {
  TIMSK1 = 0;
  millisCountdown = ms;
  TIMSK1 = (1 << OCIE1A);
}

bool send_query(byte device, const char *command, byte *buf, size_t sz) {
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
    size_t n;
    byte res;
    do {
      res = rx_data(buf, sz, &n);
      if(res == SUCCESS || res == BUFFER_FULL) {
        buf[n] = 0;
        for(size_t i = 0; i < n; ++i) if(buf[i] == '\r') buf[i] = '\n';
        Serial.print((char*)buf);
      } else {
        return 0;
      }
    } while(res == BUFFER_FULL);
    
    //return check(cmd(MSG_UNTALK));  
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
  DDRB  = DDRC  = DDRD  = 0; // all inputs  

  // Permanent direction assignments

  pinMode(TE_PIN, OUTPUT); // TE: High = Output ... this is also connected to the Nano's LED
  pinMode(PE_PIN, OUTPUT); // PE: High = Totem-Pole Output, Low = Open Collector Output
  digitalWrite(PE_PIN, HIGH);

  pinMode(ATN_PIN, OUTPUT);
  pinMode(IFC_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  digitalWrite(ATN_PIN, GPIB_FALSE);
  digitalWrite(REN_PIN, GPIB_FALSE);
  //digitalWrite(IFC_PIN, GPIB_TRUE);
  //delay(10);
  digitalWrite(IFC_PIN, GPIB_FALSE);
  
  
  pinMode(SRQ_PIN, INPUT);

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

  if(send_query(MY_SCOPE, "*IDN?", buf, RECEIVE_BUFFER_SIZE)) {
    Serial.println((char*)buf);
  }
  
  if(send_query(MY_SCOPE, "acquire?", buf, RECEIVE_BUFFER_SIZE)) {
    Serial.println((char*)buf);
  }

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
  && send_query(MY_SCOPE, "wfmpre:ch1?", buf, RECEIVE_BUFFER_SIZE)) {
    Serial.println((char*)buf);
    // 7 chars per sample, 500 samples will be 3.5K
    if(send_query(MY_SCOPE, "curve?", buf, RECEIVE_BUFFER_SIZE)) {
      Serial.println((char*)buf);
    }
  }
  
  if(send_command(MY_SCOPE, "hardcopy:format epsmono") 
  && send_command(MY_SCOPE, "hardcopy:port gpib")
  && send_query(MY_SCOPE, "hardcopy start", buf, RECEIVE_BUFFER_SIZE)) {
    Serial.println((char*)buf);
  }
}

void loop() {
}
