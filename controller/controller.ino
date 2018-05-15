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

 * ----- Transceiver Control -----------------------------------
 * PE     Pull-Up Enable             D10/PB2
 * TE     Talk Enable                D13/PB5                 L
 * DC     Direction Control          ?? -- tied Low -- means that this device is a controller
 * SC     System Controller          ?? -- tied High -- means that REN and IFC are transmitted by this controller
 * ----- Bus Data -----------------------------------
 * D1-D8  Data, terminal side        D2-D9     OUT if TE=H, otherwise IN. PE=H -> Totem-Pole Output; PE=L -> Open Collector Output
 * ----- Handshake -----------------------------------
 * DAV    Data Valid                 A1/PC1    OUT if TE high, otherwise IN
 * NRFD   (OC) Not Ready for Data    A2/PC2    IN if TE high, otherwise OUT
 * NDAC   (OC) Not Data Accepted     A3/PC3    IN if TE high, otherwise OUT
 * ----- Bus Control -----------------------------------
 * EOI    End or Identify            D11/PB3   Same as above if ATN high. Otherwise OUT if DC=L, IN if DC=H
 * ATN    Attention                  D12/PB4   Always OUTPUT (Transmit) if DC=L
 * SRQ    (OC) Service Request       A0/PC0    Always IN (Receive) if DC=L
 * IFC    Interface Clear            A4/PC4    Always OUTPUT (Transmit) if DC=L
 * REN    Remote Enable              A5/PC5    Always OUTPUT (Transmit) if DC=L
 * 
 * Directions
 *   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 | PC5 . PC4 . PC3 . PC2 . PC1 . PC0 |
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  | REN | IFC | NDAC| NRFD| DAV | SRQ |
 *   | OUT | OUT | ??? | OUT | =TE | =TE | =TE | =TE | =TE | =TE | =TE | =TE | OUT | OUT | !TE | !TE | =TE | IN  |
 * When writing command:
 *   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 |
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  |
 *   | H   | H   | X   | H   | X   | X   | X   | X   | X   | X   | X   | X   |
 *     |     |     |     |   \_____ data ____________________________________/
 *     |     |     |      \__ totem-pole outputs
 *     |     |      \__ indicate End
 *     |      \__ command
 *      \__ talk
 * 
 * 
 * 
 * Talking:
 *  -- set D1-D8 lines, TE high, ATN, EOI, PE; set DAV high (not valid)
 *  -- wait for NRFD to go high
 *  -- set DAV low
 *  -- wait for NDAC to go high
 *  -- continue with next byte
 *  -- when done talking, set TE low.
 *  
 * Listening
 *  -- set TE low, 
 *  
 * Useful commands:
 *  *ESR?   read Standard Event Status Register
 *  *STB? or serial poll     read Status Byte Register
 *  ID?
 *  *IDN?
 *  HORIZONTAL?    return horizontal settings
 *  CH1?      return vertical settings for Ch 1
 *  
 *  
 */

char *label[] = {"D1","D2","D3","D4","D5","D6","D7","D8","PE","EOI","ATN","TE","SRQ","DAV","NRFD","NDAC","IFC","REN"};

void setup() {
  pinMode(2, OUTPUT); // D1
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT); // D8
  pinMode(10, OUTPUT); // PE: High = Totem-Pole Output, Low = Open Collector Output
  pinMode(11, OUTPUT); // EOI
  pinMode(12, OUTPUT); // ATN
  pinMode(13, OUTPUT); // TE: High = Output ... this is also connected to the Nano's LED
  pinMode(14, OUTPUT); // A0 = SRQ
  pinMode(15, OUTPUT); // A1 = DAV
  pinMode(16, OUTPUT); // A2 = NRFD
  pinMode(17, OUTPUT); // A3 = NDAC
  pinMode(18, OUTPUT); // A4 = IFC
  pinMode(19, OUTPUT); // A5 = REN
  
  Serial.begin(9600);
}

void loop() {
  for(int i = 0; i < 18; ++i) {
    Serial.println(label[i]);
    for(int j = 0; j < 250; ++j) {
      digitalWrite(i+2, 1);
      delay(10);
      digitalWrite(i+2, 0);
      delay(10);
    }
  }
}
