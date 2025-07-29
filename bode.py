import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.signal import chirp, cont2discrete, lfilter
import pandas as pd


CSV_FILE = 'data/1Hz_new/05.csv'
max_freq_ = 11

# csv = pd.read_csv(CSV_FILE, sep="\t").to_numpy()
# print("Done!")
# print(csv.shape)

dir = 'data/data_2025-07-28_14-08-15/'

# === Load CSVs with no headers ===
wrench_df = pd.read_csv(dir + "actual_wrench.csv", header=None)
vel_df = pd.read_csv(dir + "velocity.csv", header=None)

force_y = wrench_df.iloc[:, 1].values
vel_y = vel_df.iloc[:, 1].values

# lim = -1
# start = 0


# times = csv[start:lim, 0]
# target_q = csv[start:lim, 1]
# target_d = np.zeros([target_q.size])
# meas_q = csv[start:lim, 2]
# meas_d = csv[start:lim, 3]
# volts_a = csv[start:lim, 4]
# volts_b = csv[start:lim, 5]
# volts_c = csv[start:lim, 6]


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


# rms_d = np.sqrt(np.mean(meas_d * meas_d))
# print(rms_d)

# plt.subplot(3, 1, 1)
# plt.plot(times, target_q)
# plt.plot(times, meas_q)
# plt.title("Requested vs Measured Current", fontsize=30)
# # plt.xlabel("Time [s]", fontsize=30)
# plt.ylabel("Current [A]", fontsize=30)
# plt.legend(["Requested", "Measured"], fontsize=30)
# # plt.show()


# plt.subplot(3, 1, 2)
# plt.plot(times, target_d)
# plt.plot(times, meas_d)
# plt.title("Requested vs Measured Current", fontsize=30)
# plt.ylabel("Current [A]", fontsize=30)
# plt.legend(["Requested", "Measured"], fontsize=30)

# plt.subplot(3, 1, 3)
# plt.plot(times, volts_a)
# plt.plot(times, volts_b)
# plt.plot(times, volts_c)
# plt.title("Phase Voltages vs Time", fontsize=30)
# plt.xlabel("Time [s]", fontsize=30)
# plt.ylabel("Phase Voltages [V]", fontsize=30)
# plt.legend(["A", "B", "C"], fontsize=30)
# plt.show()
