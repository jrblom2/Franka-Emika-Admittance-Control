import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.signal import coherence, TransferFunction, bode
import pandas as pd


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


def plot_bode(name, sets, labels, indexes, mass, damping):

    max_freq_ = 10

    dir_x = sets[0]
    dir_y = sets[1]
    dir_z = sets[2]

    # === Load CSVs with no headers ===
    wrench_df_x = pd.read_csv(dir_x + "actual_wrench.csv", header=None)
    vel_df_x = pd.read_csv(dir_x + "velocity.csv", header=None)

    force_x = wrench_df_x.iloc[:, indexes[0]].values
    vel_x = vel_df_x.iloc[:, indexes[0]].values

    wrench_df_y = pd.read_csv(dir_y + "actual_wrench.csv", header=None)
    vel_df_y = pd.read_csv(dir_y + "velocity.csv", header=None)

    force_y = wrench_df_y.iloc[:, indexes[1]].values
    vel_y = vel_df_y.iloc[:, indexes[1]].values

    wrench_df_z = pd.read_csv(dir_z + "actual_wrench.csv", header=None)
    vel_df_z = pd.read_csv(dir_z + "velocity.csv", header=None)

    force_z = wrench_df_z.iloc[:, indexes[2]].values
    vel_z = vel_df_z.iloc[:, indexes[2]].values

    fs = 1000

    # Compute frequency response
    freqs_x, H_x = compute_frequency_response(force_x, vel_x, fs)
    freqs_y, H_y = compute_frequency_response(force_y, vel_y, fs)
    freqs_z, H_z = compute_frequency_response(force_z, vel_z, fs)

    f_coh_x, Cxy_x = coherence(force_x, vel_x, fs=fs, nperseg=1024)
    f_coh_y, Cxy_y = coherence(force_y, vel_y, fs=fs, nperseg=1024)
    f_coh_z, Cxy_z = coherence(force_z, vel_z, fs=fs, nperseg=1024)

    # === Define virtual admittance parameters ===
    M = mass
    B = damping

    # === Create transfer function for ideal system: H(s) = 1 / (M s + B) ===
    num = [1.0]
    den = [M, B]
    system = TransferFunction(num, den)

    # Frequencies for theoretical plot (radians per second)
    w_ideal = 2 * np.pi * freqs_x  # Convert Hz to rad/s

    # Compute ideal bode plot at frequencies w_ideal
    _, mag_ideal, phase_ideal = bode(system, w_ideal)
    mag_linear = 10 ** (mag_ideal / 20)
    phase_rad = np.deg2rad(phase_ideal)

    # Plot magnitude and phase
    plt.figure(figsize=(12, 9))

    plt.subplot(3, 1, 1)
    plt.loglog(freqs_x, np.abs(H_x), label=labels[0])
    plt.loglog(freqs_y, np.abs(H_y), label=labels[1])
    plt.loglog(freqs_z, np.abs(H_z), label=labels[2])
    plt.loglog(freqs_x, mag_linear, '--', label='Expected (m=1.0, c=1.0)', color='black')
    plt.legend()
    plt.title("Frequency Response")
    plt.ylabel("Amplitude Ratio")
    plt.xlim([0.1, max_freq_])
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.semilogx(freqs_x, np.angle(H_x), label=labels[0])
    plt.semilogx(freqs_y, np.angle(H_y), label=labels[1])
    plt.semilogx(freqs_z, np.angle(H_z), label=labels[2])
    plt.semilogx(freqs_x, phase_rad, '--', label='Expected', color='black')
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Phase (radians)")
    plt.legend()
    plt.xlim([0.1, max_freq_])
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.semilogx(f_coh_x, Cxy_x, label=labels[0])
    plt.semilogx(f_coh_y, Cxy_y, label=labels[1])
    plt.semilogx(f_coh_z, Cxy_z, label=labels[2])
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Coherence")
    plt.xlim([0.1, max_freq_])
    plt.ylim([0, 1.05])
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'bode_{name}.png', dpi=500, bbox_inches='tight')
    plt.show()


labels = ['X', 'Y', 'Z']
indexes = [0, 1, 2]
sets = ['data/data_2025-07-29_12-21-38/', 'data/data_2025-07-29_12-19-20/', 'data/data_2025-07-29_12-23-30/']
plot_bode('xyz', sets, labels, indexes, 1.0, 1.0)

labels = ['Roll', 'Pitch', 'Yaw']
indexes = [3, 4, 5]
sets = ['data/data_2025-07-29_12-50-12/', 'data/data_2025-07-29_12-48-13/', 'data/data_2025-07-29_12-44-29/']
plot_bode('rpy', sets, labels, indexes, 1.0, 0.1)
