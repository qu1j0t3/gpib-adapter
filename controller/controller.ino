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

#define D1_PIN    2
#define D2_PIN    3
#define D3_PIN    4
#define D4_PIN    5
#define D5_PIN    6
#define D6_PIN    7
#define D7_PIN    8
#define D8_PIN    9
#define PE_PIN   10
#define EOI_PIN  11
#define ATN_PIN  12
#define TE_PIN   13
#define SRQ_PIN  14
#define DAV_PIN  15
#define NRFD_PIN 16
#define NDAC_PIN 17
#define IFC_PIN  18
#define REN_PIN  19
  
#define TE_TALK     HIGH
#define TE_LISTEN   LOW

// GPIB levels are signal sense inverted:
#define GPIB_FALSE   HIGH
#define GPIB_TRUE    LOW
  
void mode(bool talk, bool atn){
  // SAFETY: Arduino outputs connected to transceiver lines configured as outputs are
  //         likely to violate driver current limits. Set all Arduino pins to high-Z,
  //         so that none are outputs when transceiver lines are switched to outputs by
  //         the change in Talk Enable line.
          
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

/*   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 | PC5 . PC4 . PC3 . PC2 . PC1 . PC0 |   Port/Pin
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  | REN | IFC | NDAC| NRFD| DAV | SRQ |   Bus
 */
 
#define PC_SRQ_MASK       1

#define PB_TE_MASK        0b100000
#define PB_ATN_FALSE_MASK  0b10000  // combine these three bits
#define PB_EOI_FALSE_MASK   0b1000  // to form `mask` parameter
#define PB_PE_TOTEMPOLE      0b100  // for transmit

// multiline message with ATN FALSE (device dependent data)
#define DATA_NO_EOI  (PB_PE_TOTEMPOLE | PB_ATN_FALSE_MASK | PB_EOI_FALSE_MASK)
#define DATA_EOI     (PB_PE_TOTEMPOLE | PB_ATN_FALSE_MASK)

#define ERR      0
#define SUCCESS  1
#define EOI      2 // if a received byte is EOI
#define TIMEOUT  3 // not impl yet
#define SRQ      4


// X=don't care (received message); X=shall not drive (transmitted)
// Y=don't care (transmitted);      Y=don't care (received)
  //                                 Type Class /-----DIO-----\ VRA ATN EOI SRQ IFC REN
  // MLA: My Listen Address        = M    AD    Y 0 1 L L L L L XXX 1   X   X   X   X
  // MTA: My Talk Address          = M    AD    Y 1 0 T T T T T XXX 1   X   X   X   X
  // TAG: Talk Address Group       = M    AD    Y 1 0 X X X X X XXX 1   X   X   X   X
  // send scope talk address OTA?  = TAG & MTA
  // send command?

// Remote message definitions per IEEE 488.1-1987 section 2.13

// AD - Address (talk or listen)
#define MSG_LISTEN        0b0100000 // 0 1 L L L L L
#define MSG_UNLISTEN      0b0111111
#define MSG_TALK          0b1000000 // 1 0 T T T T T
#define MSG_UNTALK        0b1011111
// SE - Secondary
#define MSG_SEC_ADDRESS   0b1100000 // 1 1 S S S S S
#define MSG_PAR_POLL_ENB  0b1100000 // 1 1 0 S P P P
#define MSG_PAR_POLL_DIS  0b1110000 // 1 1 1 D D D D - send zeroes for D
// AC - Addressed command
#define MSG_GO_TO_LOCAL   0b0000001
#define MSG_SEL_DEV_CLR   0b0000100
#define MSG_PAR_POLL_CFG  0b0000101
#define MSG_GRP_EXEC_TRG  0b0001000
#define MSG_TAKE_CONTROL  0b0001001
// UC - Universal command
#define MSG_LOCALLOCKOUT  0b0010001
#define MSG_DEVICE_CLEAR  0b0010100
#define MSG_PAR_POLL_UNC  0b0010101
#define MSG_SER_POLL_ENB  0b0011000
#define MSG_SER_POLL_DIS  0b0011001
  
byte transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return ERR;
  } else if(!(PINC & PC_SRQ_MASK)) {
    return SRQ;
  }
  
  digitalWrite(DAV_PIN, HIGH); // data not valid
  
  if(digitalRead(NRFD_PIN) && digitalRead(NDAC_PIN)) {
    return ERR;
  } else {
    value = ~ value;
    PORTB = PB_TE_MASK | mask | (value >> 6);
    PORTD = value << 2;

    delayMicroseconds(2);
    
    // Wait until all acceptors are ready for data
    while(!digitalRead(NRFD_PIN))
      ;
      
    digitalWrite(DAV_PIN, LOW); // data valid
    
    // Wait until all acceptors have accepted the data
    while(!digitalRead(NDAC_PIN))
      ;
      
    digitalWrite(DAV_PIN, HIGH); // data not valid
    return SUCCESS;
  }
}

byte receive(byte *value){
  if(PORTB & PB_TE_MASK) { // interface must be in Listen direction
    return ERR;
  } else if(!(PINC & PC_SRQ_MASK)) {
    return SRQ;
  }
  
  digitalWrite(NDAC_PIN, LOW);  // not accepted data
  digitalWrite(NRFD_PIN, HIGH); // ready for data

  // wait for data valid to go low (true)
  while(digitalRead(DAV_PIN))
    ;

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

size_t rx_data(byte *buf, size_t max) {
  mode(TE_LISTEN, /*ATN*/GPIB_FALSE);
  size_t n = 0;
  for(byte *p = buf; n < max; ++n, ++p) {
    byte res = receive(p);
    if(res == EOI || *p == '\n') { // FIXME: What about binary data
      return ++n;
    } else if(res != SUCCESS) {
      break; // TODO: Report error
    }
  }
  return n;
}

#define RECEIVE_BUFFER_SIZE 0x200

byte cmd(byte msg) {
  mode(TE_TALK, /*ATN*/GPIB_TRUE);
  return transmit(PB_PE_TOTEMPOLE | PB_EOI_FALSE_MASK, msg);
}

#define MY_SCOPE 2

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

bool check(byte res) {
  switch(res) {
    case EOI:
    case SUCCESS:
      return 1;
    case ERR:
      Serial.println("Transmit error (NRFD & NDAC)\n"); 
      return 0;
    case SRQ:
      Serial.println("Request service\n");
      //serialpoll();
      return 0;
    case TIMEOUT: 
      Serial.println("Timeout\n"); 
      return 0;
  }
  return 0;
}

void output_test(byte num) {
  Serial.print("\n\n\nRunning OUTPUT test ");
  Serial.print(num);
  Serial.println(". ADAPTER SHOULD BE DISCONNECTED FROM GPIB BUS.");
  Serial.println("Test the specified Arduino pins with Oscilloscope or DVM.");
  Serial.println("Oscilloscope should show square wave. DVM should read about half Vcc, or approx 2.5V.");
  
  if(num == 1) {
    Serial.println("Transceiver control outputs: TE (D13), PE (D10)");
  } else if(num == 2) {
    Serial.println("Bus outputs: ATN (D12), IFC (A4), REN (A5)");
  } else if(num == 3) {
    Serial.println("Bidirectional pins: B1-B8 (Arduino D2-D9), EOI (D11), DAV (A1)");
  } else if(num == 4) {       // Test handshaking pins as outputs when TE is low
    Serial.println("Handshaking pins: NRFD (A2), NDAC (A3)");
  }
  
  for(bool x = HIGH;; x ^= 1) {
    if(num == 1) {              // Test control outputs
      digitalWrite(TE_PIN, x);
      digitalWrite(PE_PIN, x);
    } else if(num == 2) {       // Test bus outputs
      digitalWrite(ATN_PIN, x);
      digitalWrite(IFC_PIN, x);
      digitalWrite(REN_PIN, x);
    } else if(num == 3) {       // Test bidir pins as outputs
      digitalWrite(PE_PIN,  HIGH);
      mode(/*TE*/ HIGH, /*ATN*/ HIGH);
      digitalWrite(D1_PIN,  x);
      digitalWrite(D2_PIN,  x);
      digitalWrite(D3_PIN,  x);
      digitalWrite(D4_PIN,  x);
      digitalWrite(D5_PIN,  x);
      digitalWrite(D6_PIN,  x);
      digitalWrite(D7_PIN,  x);
      digitalWrite(D8_PIN,  x);
      digitalWrite(EOI_PIN, x);
      digitalWrite(DAV_PIN, x);
    } else if(num == 4) {       // Test handshaking pins as outputs when TE is low
      digitalWrite(PE_PIN,  HIGH);
      mode(/*TE*/ LOW, /*ATN*/ HIGH);
      digitalWrite(NRFD_PIN, x);
      digitalWrite(NDAC_PIN, x);
    }
    delay(1);
  }
}

void input_test(byte num) {
  Serial.print("\n\n\nRunning INPUT test ");
  Serial.print(num);
  Serial.println(". ADAPTER SHOULD BE DISCONNECTED FROM GPIB BUS.");
  Serial.println("When disconnected, transceivers should read signals as HIGH (1).");
  Serial.println("To carry out the test, short each specified BUS-side input to GND");
  Serial.println("and observe the Arduino reads it as 0.");
  Serial.println("(The Arduino GPIO pins (terminal side) can't be tested directly with the transceiver in circuit.)");
  
  if(num == 1) {
    Serial.println("Test bus side SRQ.");
    
    for(byte x = 0;; ++x) {
      Serial.print(x);
      Serial.print("  SRQ: ");
      Serial.println(digitalRead(SRQ_PIN));
      delay(500);
    }
  } else if(num == 2) {
    Serial.println("Test bus side NRFD & NDAC.");
    
    digitalWrite(PE_PIN,  HIGH);
    mode(/*TE*/ HIGH, /*ATN*/ HIGH);
    for(byte x = 0;; ++x) {
      Serial.print(x);
      Serial.print("  NRFD: ");
      Serial.print(digitalRead(NRFD_PIN));
      Serial.print("  NDAC: ");
      Serial.println(digitalRead(NDAC_PIN));
      delay(500);
    }
  } else if(num == 3) {
    Serial.println("Test bus side B1 through B8, EOI, DAV.");
    
    digitalWrite(PE_PIN,  HIGH);
    mode(/*TE*/ LOW, /*ATN*/ HIGH);
    for(byte x = 0;; ++x) {
      Serial.print(x);
      Serial.print("  DATA: ");
      Serial.print(digitalRead(D8_PIN));
      Serial.print(digitalRead(D7_PIN));
      Serial.print(digitalRead(D6_PIN));
      Serial.print(digitalRead(D5_PIN));
      Serial.print(digitalRead(D4_PIN));
      Serial.print(digitalRead(D3_PIN));
      Serial.print(digitalRead(D2_PIN));
      Serial.print(digitalRead(D1_PIN));
      Serial.print("  EOI: ");
      Serial.print(digitalRead(EOI_PIN));
      Serial.print("  DAV: ");
      Serial.println(digitalRead(DAV_PIN));
      delay(500);
    }
  }
}

void eight_bits(byte v, char *s) {
  for(byte m = 0x80; m; m >>= 1) {
    *s++ = v & m ? '1' : '0';
  }
  *s = 0;
}

void setup() {
  DDRB  = DDRC  = DDRD  = 0; // all inputs  

  // Permanent direction assignments

  pinMode(TE_PIN, OUTPUT); // TE: High = Output ... this is also connected to the Nano's LED
  pinMode(PE_PIN, OUTPUT); // PE: High = Totem-Pole Output, Low = Open Collector Output
  digitalWrite(PE_PIN, HIGH);

  pinMode(ATN_PIN, OUTPUT);
  digitalWrite(ATN_PIN, HIGH); // ATN False
  pinMode(IFC_PIN, OUTPUT);
  digitalWrite(IFC_PIN, HIGH); // IFC False
  pinMode(REN_PIN, OUTPUT);
  digitalWrite(REN_PIN, HIGH); // REN False
  
  pinMode(SRQ_PIN, INPUT);

  Serial.begin(115200);

  //output_test(1);
  //input_test(1);
  
  //mode(/*TE*/LOW, /*ATN*/HIGH);
  /*for(byte n=10; n--;) {
    char x[9];
    eight_bits(PINC, x);
    Serial.print(n);
    Serial.print("  ");
    Serial.println(x);
    delay(500);
  }*/

  Serial.println("Controller start");

  byte device = MY_SCOPE;
  byte buf[RECEIVE_BUFFER_SIZE+1];
  
check_srq:
  while(!(PINC & PC_SRQ_MASK)) {
    serialpoll();
    delay(1000);
  }
  
  if( check(cmd(MSG_UNLISTEN)) &&
      check(cmd(MSG_LISTEN | device)) &&
      check(tx_data("*IDN?")) &&
      check(cmd(MSG_UNLISTEN)) &&
      check(cmd(MSG_UNTALK)) &&
      check(cmd(MSG_TALK | device)) )
  {
    size_t n = rx_data(buf, RECEIVE_BUFFER_SIZE);
    buf[n] = 0;
    Serial.println((char*)buf);
    
    check(cmd(MSG_UNTALK));  
  } else {
    goto check_srq;
  }

}

void loop() {
  
}
