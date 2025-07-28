import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch, csd, TransferFunction, bode

dir = 'data/data_2025-07-28_12-55-55/'

# === Load CSVs with no headers ===
wrench_df = pd.read_csv(dir + "actual_wrench.csv", header=None)
vel_df = pd.read_csv(dir + "velocity.csv", header=None)

# === Sampling settings ===
dt = 0.01  # Time step in seconds
fs = 1.0 / dt  # Sampling frequency in Hz
N = len(wrench_df)

# === Reconstruct time vector ===
time = np.arange(N) * dt

# === Extract column 1 (Y-axis) ===
force_y = wrench_df.iloc[:, 1].values
vel_y = vel_df.iloc[:, 1].values

# === Estimate transfer function H(f) = Pxy / Pxx ===
f, Pxy = csd(vel_y, force_y, fs=fs, nperseg=1024)
_, Pxx = welch(force_y, fs=fs, nperseg=1024)
H = Pxy / (Pxx + 1e-12)  # Avoid divide-by-zero

# === Bode plot data (measured) ===
magnitude = 20 * np.log10(np.abs(H))
phase = np.angle(H, deg=True)

# === Define virtual admittance parameters ===
M = 0.75  # virtual mass (kg), adjust as needed
B = 5.0  # virtual damping (N.s/m), adjust as needed

# === Create transfer function for ideal system: H(s) = 1 / (M s + B) ===
num = [1.0]
den = [M, B]
system = TransferFunction(num, den)

# Frequencies for theoretical plot (radians per second)
w_ideal = 2 * np.pi * f  # Convert Hz to rad/s

# Compute ideal bode plot at frequencies w_ideal
_, mag_ideal, phase_ideal = bode(system, w_ideal)

# === Plot both measured and ideal bode plots ===
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.semilogx(f, magnitude, label='Measured', color='blue')
plt.semilogx(f, mag_ideal, label='Ideal (1/(Ms+B))', color='orange', linestyle='--')
plt.ylabel("Magnitude (dB)")
plt.title("Bode Plot: Force_Y → Velocity_Y")
plt.grid(True, which="both")
plt.legend()

plt.subplot(2, 1, 2)
plt.semilogx(f, phase, label='Measured', color='blue')
plt.semilogx(f, phase_ideal, label='Ideal (1/(Ms+B))', color='orange', linestyle='--')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (degrees)")
plt.grid(True, which="both")
plt.legend()

plt.tight_layout()
plt.show()
