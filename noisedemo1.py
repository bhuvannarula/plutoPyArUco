import numpy as np
from math import pi

arr = np.load('noisem.npy')

from scipy.signal import butter, lfilter, sosfilt

order = 1
fs = 30
nyq = 0.5 * fs
lowcut = 3 #1.6
low = lowcut / nyq
b, a = butter(order, low, btype='low')

record = arr

from scipy.signal import iirnotch, filtfilt

samp_freq = 30
notch_freq = 15
quality_factor = 1.0375
b_notch, a_notch = iirnotch(notch_freq, quality_factor, samp_freq)
sos = butter(2, [0.00001, 1.5], analog=False, btype='band', output='sos', fs=samp_freq)

import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.plot(record, color='r')


filtered = filtfilt(b_notch, a_notch, record)

filtered = sosfilt(sos, record)
filtered = lfilter(b, a, filtered)
#filtered = record - filtered
ax.plot(filtered, color='g')

plt.show()
