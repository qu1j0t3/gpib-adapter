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

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from math import sqrt, copysign, modf, log2, ceil, pow
from skimage.io import imread

# This is designed to extract curve data from an image
# for FFT measurement and THD estimation.

# The sample image badsine-horn2.png is taken from
# "Oscillators Simplified" by Delton T. Horn (1987, TAB Books).

# The image should be white plot on black background.
# The curve should extend from left edge to right edge
# and must include at least one complete cycle.
# There must be no extraneous marks other than the curve itself.
# The cycle between first and second positive-going crossings
# (left to right) will be captured. (If the curve is negative going
# at leftmost crossing, then flip the image vertically 
# before processing.) It will then be upsampled
# (with bilinear interpolation) to the next power-of-2 samples,
# before a DFT is performed.

img = imread('badsine-horn2.png', True).transpose()
c1 = [sum([i*v for i,v in enumerate(col)]) / sum([v for i,v in enumerate(col)])
     for col in img]
# shift positive curve to AC so that zero crossings can be found
mn = (np.min(c1) + np.max(c1))/2
curve = [x-mn for x in c1]


# find zero crossings (positive-going)
# (if the signal is noisy, it should be filtered before doing this)
c1,c2 = [i for i in range(len(curve)-1) if curve[i] < 0 and curve[i+1] > 0]
t0 = c1 - curve[c1] / (curve[c1+1] - curve[c1])
t1 = c2 - curve[c2] / (curve[c2+1] - curve[c2])
print("t0 = {}, t1 = {}   Samples per cycle: {}".format(t0, t1, t1-t0))
cycle = curve[c1:c2+1]
print("Sampled curve has {} points.  Min: {}  Max: {}".format(len(curve), min(cycle), max(cycle)))

# Compute an interpolated sample for any continuous time along the curve.
# This will be used to upsample the curve period to a power of 2 samples.
def interp_sample(t):
  f, i = modf(t)
  return curve[int(i)] + f*(curve[int(i)+1] - curve[int(i)])

exp = ceil(log2(len(curve)))
pts = int(pow(2, exp))
print("Next power of 2 = {}  ({} points)".format(exp, pts))
# Subtract mean value, in order to zero DC component
mn = np.mean(curve)
resampled = [interp_sample(t0 + (t/float(pts))*(t1-t0)) - mn for t in range(pts)]

#resampled = [copysign(1, x) for x in resampled] # fake a square wave for testing ~ 48% THD
fft = np.fft.rfft(resampled).imag  # sine basis functions are in imaginary axis (cosines are real)
thd = -sqrt(sum([x*x for x in fft[2:]])) / fft[1]
print("THD = {}%".format(thd*100))
print(fft)

plt.bar(range(30), fft[0:30], tick_label=range(30))
plt.show()

#plt.plot(resampled)
#plt.show()

# Reconstruct signal from first five harmonics
mod_fft = np.zeros_like(fft, np.complex)
mod_fft[1] = complex(0,fft[1])
f = mod_fft.copy()
mod_fft[2] = complex(0,fft[2])
mod_fft[3] = complex(0,fft[3])
mod_fft[4] = complex(0,fft[4])
mod_fft[5] = complex(0,fft[5])
#plt.plot(resampled)
reconstruct = np.fft.irfft(mod_fft).real
fundamental = np.fft.irfft(f).real

# Show reconstructed signal, the fundamental,
# along with the original signal and an error term
# between reconstructed and original.
plt.plot(reconstruct)
plt.plot(fundamental)
plt.plot(resampled)
plt.plot(reconstruct - resampled)
plt.show()
