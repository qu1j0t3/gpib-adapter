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
#define ATN_FALSE   HIGH
#define ATN_TRUE    LOW

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
  
  byte fwd = talk ? OUTPUT : INPUT;
  byte rev = talk ? INPUT  : OUTPUT;

  digitalWrite(TE_PIN, talk); // Talk Enable
  
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
}

/*   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 | PC5 . PC4 . PC3 . PC2 . PC1 . PC0 |   Port/Pin
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  | REN | IFC | NDAC| NRFD| DAV | SRQ |   Bus
 */
 
#define PB_TE_MASK        0b100000

#define PB_ATN_FALSE_MASK  0b10000  // combine these three bits
#define PB_EOI_FALSE_MASK   0b1000  // to form `mask` parameter
#define PB_PE_TOTEMPOLE      0b100  // for transmit

#define ERR      0
#define SUCCESS  1
#define EOI      2 // if a received byte is EOI
#define TIMEOUT  3 // not impl yet

bool transmit(byte mask, byte value){
  if(!(PORTB & PB_TE_MASK)) { // interface must be in Talk direction
    return ERR;
  }
  
  digitalWrite(DAV_PIN, HIGH); // data not valid
  
  if(digitalRead(NRFD_PIN) && digitalRead(NDAC_PIN)) {
    return ERR;
  } else {
    PORTB = PB_TE_MASK | mask | (value >> 6);
    PORTD = value << 2;
    
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

bool receive(byte *value){
  if(PORTB & PB_TE_MASK) { // interface must be in Listen direction
    return ERR;
  }
  
  digitalWrite(NDAC_PIN, LOW);  // not accepted data
  digitalWrite(NRFD_PIN, HIGH); // ready for data

  // wait for data valid to go low (true)
  while(digitalRead(DAV_PIN))
    ;

  digitalWrite(NRFD_PIN, LOW);  // not ready for data
  
  *value = (PORTB << 6) | (PORTD >> 2);
  bool eoi_state = !digitalRead(EOI_PIN);
  
  digitalWrite(NDAC_PIN, HIGH); // accepted data
  
  return eoi_state ? EOI : SUCCESS;
}

void cmd(){
  mode(TE_TALK, ATN_TRUE);
  if(!transmit(PB_EOI_FALSE_MASK | PB_PE_TOTEMPOLE, 0)) {
    Serial.println("Error: NRFD & NDAC high. Aborted transmit.");
  }
}

void setup() {
  PORTB = PORTC = PORTD = 0; // all low
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

  mode(TE_LISTEN, ATN_FALSE);

  Serial.begin(115200);
}

void loop() {
  
}
