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

# Sampled curve data from scope

curve = [-23924,-23781,-23585,-23428,-23185,-22995,-22772,-22552,-22253,-21889,-21624,-21352,-21084,-20664,-20352,-19988,-19592,-19276,-18811,-18404,-18007,-17573,-17128,-16655,-16209,-15643,-15136,-14597,-14120,-13595,-13050,-12480,-12009,-11389,-10822,-10204,-9658,-9084,-8552,-7985,-7341,-6787,-6143,-5551,-4942,-4353,-3738,-3075,-2406,-1796,-1153,-524,81,779,1418,2080,2647,3336,3979,4622,5240,5860,6413,7073,7630,8229,8837,9416,9975,10505,11146,11693,12235,12759,13349,13847,14403,14931,15405,15971,16431,16918,17361,17777,18269,18639,19091,19498,19945,20311,20690,21097,21367,21728,22040,22349,22607,22899,23132,23352,23563,23784,23999,24146,24306,24436,24554,24689,24788,24838,24855,24928,24974,24993,25006,24988,24946,24872,24841,24796,24718,24614,24490,24347,24187,24035,23826,23615,23396,23214,22953,22703,22404,22140,21819,21450,21167,20794,20394,20074,19590,19181,18757,18364,17920,17445,17021,16552,16138,15566,15004,14554,14036,13448,12897,12367,11849,11272,10682,10108,9579,8996,8403,7822,7185,6598,5997,5385,4794,4107,3511,2801,2219,1594,927,273,-303,-1020,-1654,-2267,-2902,-3570,-4207,-4814,-5384,-6011,-6651,-7233,-7781,-8364,-8952,-9518,-10062,-10632,-11249,-11828,-12346,-12854,-13454,-13992,-14463,-14994,-15546,-16015,-16556,-17000,-17507,-17883,-18287,-18710,-19160,-19516,-19900,-20262,-20604,-20940,-21288,-21550,-21825,-22207,-22456,-22708,-22952,-23146,-23380,-23570,-23745,-23872,-24055,-24119,-24247,-24369,-24447,-24531,-24567,-24580,-24595,-24582,-24589,-24552,-24541,-24511,-24416,-24328,-24182,-24070,-23967,-23811,-23658,-23494,-23281,-23062,-22843,-22625,-22360,-21989,-21691,-21437,-21156,-20762,-20465,-20070,-19757,-19351,-18963,-18508,-18127,-17696,-17281,-16796,-16371,-15796,-15288,-14771,-14281,-13752,-13220,-12636,-12146,-11569,-11016,-10391,-9859,-9257,-8748,-8169,-7515,-6976,-6372,-5717,-5125,-4556,-3928,-3295,-2595,-1994,-1349,-722,-96,562,1225,1907,2470,3114,3786,4381,5067,5682,6234,6886,7459,8063,8626,9262,9811,10327,10947,11503,12088,12568,13156,13701,14234,14772,15232,15799,16299,16769,17222,17628,18115,18507,18958,19347,19812,20201,20530,20970,21291,21623,21953,22237,22539,22807,23071,23291,23488,23715,23937,24091,24230,24397,24530,24657,24767,24828,24845,24901,24980,24982,24986,24989,24945,24914,24856,24828,24772,24649,24529,24398,24233,24060,23881,23681,23467,23272,23012,22763,22490,22221,21919,21574,21239,20927,20497,20173,19745,19288,18888,18502,18083,17592,17134,16735,16273,15750,15167,14714,14187,13638,13067,12515,12018,11453,10858,10277,9755,9178,8573,7992,7400,6774,6153,5614,4992,4342,3707,3004,2417,1807,1121,498,-145,-783,-1463,-2094,-2703,-3393,-4005,-4607,-5198,-5789,-6426,-7068,-7593,-8204,-8781,-9318,-9934,-10454,-11073,-11654,-12209,-12687,-13298,-13823,-14340,-14822,-15384,-15875,-16424,-16815,-17379,-17742,-18166,-18574,-19016,-19419,-19790,-20145,-20480,-20790,-21221,-21499,-21725,-22072,-22387,-22625,-22853,-23058,-23286,-23524,-23668,-23816,-23996,-24083,-24204,-24328,-24425,-24520,-24554,-24576,-24593,-24591,-24595,-24579,-24561,-24530,-24443,-24338,-24246,-24104,-24024,-23837,-23736,-23562,-23337,-23127,-22930,-22693
]

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

plt.bar(range(len(fft)), fft, tick_label=range(len(fft)))
plt.show()
