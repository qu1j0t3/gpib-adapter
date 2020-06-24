#!/bin/bash

#    This file is part of "GPIB Adapter", an Arduino based controller for GPIB (IEEE 488).
#    Copyright (C) 2018 Toby Thain, toby@telegraphics.com.au
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

set -e

# Tested on ???
# Based on asjackson's https://github.com/AaronJackson/gpibtools/blob/master/tek_acq.sh

PORT=${1:-/dev/ttyUSB0}
ADDR=${2:-1}

stty -F $PORT 115200 clocal -opost -isig -icanon -echo min 1 time 100

{GPIB_IN}<$PORT

# Flush out banner. Read until timeout
while read -u $GPIB_IN -t 7 DATA ; do : ; done

(
    echo "++addr $ADDR" 

    echo "pu;pa500,300;pd;"
    echo "pr0,1000,1000,0,0,-1000,-1000,0;"

) > $PORT
