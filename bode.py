import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.signal import chirp, cont2discrete, lfilter
import pandas as pd

max_freq_ = 11

dir = 'data/data_2025-07-28_14-08-15/'

# === Load CSVs with no headers ===
wrench_df = pd.read_csv(dir + "actual_wrench.csv", header=None)
vel_df = pd.read_csv(dir + "velocity.csv", header=None)

force_y = wrench_df.iloc[:, 1].values
vel_y = vel_df.iloc[:, 1].values


def compute_frequency_response(input_signal, output_signal, fs):
    """
    Computes frequency response H(f) = Y(f) / X(f)
    input_signal: Excitation chirp
    output_signal: System response to chirp
    fs: Sampling frequency
    """
    n = len(input_signal)
    # Ensure both signals are the same length
    input_signal = input_signal[:n]
    output_signal = output_signal[:n]

    # FFT of input and output
    X = np.fft.fft(input_signal)
    Y = np.fft.fft(output_signal)

    # Frequency response
    H = Y / X

    # Frequency vector
    freqs = np.fft.fftfreq(n, d=1 / fs)

    return freqs[: n // 2], H[: n // 2]  # Return only positive frequencies


fs = 1000

# Compute frequency response
freqs, H = compute_frequency_response(force_y, vel_y, fs)

# Plot magnitude and phase
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.loglog(freqs, np.abs(H))
plt.title("Frequency Response")
plt.ylabel("Amplitude Ratio")
plt.xlim([0.1, max_freq_])
plt.grid(True)

plt.subplot(2, 1, 2)
plt.semilogx(freqs, np.angle(H))
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (radians)")
plt.xlim([0.1, max_freq_])
plt.grid(True)

plt.tight_layout()
plt.show()
