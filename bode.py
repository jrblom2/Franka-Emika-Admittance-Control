import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.signal import chirp, cont2discrete, lfilter
import pandas as pd

max_freq_ = 100

dir_x = 'data/data_2025-07-29_12-21-38/'  # X
dir_y = 'data/data_2025-07-29_12-19-20/'  # Y
dir_z = 'data/data_2025-07-29_12-23-30/'  # Z

# === Load CSVs with no headers ===
wrench_df_x = pd.read_csv(dir_x + "actual_wrench.csv", header=None)
vel_df_x = pd.read_csv(dir_x + "velocity.csv", header=None)

force_x = wrench_df_x.iloc[:, 0].values
vel_x = vel_df_x.iloc[:, 0].values

wrench_df_y = pd.read_csv(dir_y + "actual_wrench.csv", header=None)
vel_df_y = pd.read_csv(dir_y + "velocity.csv", header=None)

force_y = wrench_df_y.iloc[:, 1].values
vel_y = vel_df_y.iloc[:, 1].values

wrench_df_z = pd.read_csv(dir_z + "actual_wrench.csv", header=None)
vel_df_z = pd.read_csv(dir_z + "velocity.csv", header=None)

force_z = wrench_df_z.iloc[:, 2].values
vel_z = vel_df_z.iloc[:, 2].values


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
freqs_x, H_x = compute_frequency_response(force_x, vel_x, fs)
freqs_y, H_y = compute_frequency_response(force_y, vel_y, fs)
freqs_z, H_z = compute_frequency_response(force_z, vel_z, fs)

# Plot magnitude and phase
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.loglog(freqs_x, np.abs(H_x), label='X')
plt.loglog(freqs_y, np.abs(H_y), label='Y')
plt.loglog(freqs_z, np.abs(H_z), label='Z')
plt.legend()
plt.title("Frequency Response")
plt.ylabel("Amplitude Ratio")
plt.xlim([0.1, max_freq_])
plt.grid(True)

plt.subplot(2, 1, 2)
plt.semilogx(freqs_x, np.angle(H_x), label='X')
plt.semilogx(freqs_y, np.angle(H_y), label='Y')
plt.semilogx(freqs_z, np.angle(H_z), label='Z')
plt.legend()
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (radians)")
plt.xlim([0.1, max_freq_])
plt.grid(True)

plt.tight_layout()
plt.show()
