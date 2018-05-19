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
  
  digitalWrite(10, 1); // PE
  digitalWrite(13, 1); // TE
  
  Serial.begin(9600);
}

void loop() {
  for(int i = 0; i < 18; ++i) {
    Serial.println(label[i]);

    if(i != 8 && i != 11) { // skip PE and TE
      digitalWrite(i+2, 1);
      while(Serial.read() != 13)
        ;
      digitalWrite(i+2, 0);
    }
  }
}
