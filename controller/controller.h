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

#define TE_TALK     HIGH
#define TE_LISTEN   LOW

// GPIB levels are signal sense inverted:
#define GPIB_FALSE   HIGH
#define GPIB_TRUE    LOW


/*   | PB5 . PB4 . PB3 . PB2 . PB1 . PB0 | PD7 . PD6 . PD5 . PD4 . PD3 . PD2 | PC5 . PC4 . PC3 . PC2 . PC1 . PC0 |   Port/Pin
 *   | TE  | ATN | EOI | PE  | D8  . D7  . D6  . D5  . D4  . D3  . D2  . D1  | REN | IFC | NDAC| NRFD| DAV | SRQ |   Bus
 */
 

#define PB_TE_MASK   0b100000
#define PB_ATN_MASK   0b10000  // combine these three bits
#define PB_EOI_MASK    0b1000  // to form `mask` parameter
#define PB_PE_TOTEMPOLE 0b100  // for transmit

#define PC_REN_MASK  0b100000
#define PC_IFC_MASK   0b10000
#define PC_NDAC_MASK   0b1000
#define PC_NRFD_MASK    0b100
#define PC_DAV_MASK      0b10
#define PC_SRQ_MASK         1

// multiline message with ATN FALSE (device dependent data)
#define DATA_NO_EOI  (PB_PE_TOTEMPOLE | PB_ATN_MASK | PB_EOI_MASK)
#define CMD_NO_EOI   (PB_PE_TOTEMPOLE | PB_EOI_MASK)
#define DATA_EOI     (PB_PE_TOTEMPOLE | PB_ATN_MASK)

#define ERR         0
#define SUCCESS     1
#define EOI         2 // if a received byte is EOI
#define TIMEOUT     3
#define SRQ         4
#define BUFFER_FULL 5

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

#define NO_DEVICE 31

void mode(bool talk, byte atn_eoi_pe);

