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
#include <Arduino.h>

#include "controller.h"
#include "pinmap.h"

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

