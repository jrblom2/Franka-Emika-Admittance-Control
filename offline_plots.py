import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np

# Argument parsing
parser = argparse.ArgumentParser(description='Plot data from CSV files.')
parser.add_argument('data_dir', help='Path to the data directory (containing CSVs)')
args = parser.parse_args()

data_dir = args.data_dir

# Constants
output_dir = os.path.join(data_dir, 'plots')
os.makedirs(output_dir, exist_ok=True)
sampling_rate = 100


# Generic single file plotter
def plot_data(file_name, labels, ylabel, num_columns, y_lim_min=None, y_lim_max=None):
    df = pd.read_csv(os.path.join(data_dir, file_name + '.csv'), header=None)
    time = df.index / sampling_rate

    plt.figure()
    for i in range(num_columns):
        plt.plot(time, df[i], label=labels[i])
    plt.title(f'Plot for {file_name}')
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.ylim((y_lim_min, y_lim_max))
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f'{file_name}.png'))
    plt.close()


def plot_data_with_error_from_zero(file_name, labels, ylabel, num_columns, y_lim_min=None, y_lim_max=None):
    df = pd.read_csv(os.path.join(data_dir, file_name + '.csv'), header=None)
    time = df.index / sampling_rate

    # Create a 1x3 layout: [original signals | error magnitude | cumulative error]
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))

    # Plot all signals in the first subplot
    for i in range(num_columns):
        axs[0].plot(time, df[i] * (180 / np.pi), label=labels[i])
    axs[0].set_title(f'{file_name} - Signals')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel(ylabel)
    if y_lim_min is not None and y_lim_max is not None:
        axs[0].set_ylim((y_lim_min, y_lim_max))
    axs[0].legend()
    axs[0].grid(True)

    # Compute error from zero vector
    error_magnitude = np.linalg.norm(df.values, axis=1)
    cumulative_error = np.cumsum(error_magnitude)

    # Plot error magnitude
    axs[1].plot(time, error_magnitude, color='red')
    axs[1].set_title('Error Magnitude (from Zero)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Euclidean Distance')
    axs[1].grid(True)

    # Plot cumulative error
    axs[2].plot(time, cumulative_error, color='purple')
    axs[2].set_title('Cumulative Error (from Zero)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Cumulative Distance')
    axs[2].grid(True)

    fig.tight_layout(pad=4.0)
    plt.savefig(os.path.join(output_dir, f'{file_name}.png'))
    plt.close()


def plot_comparison(file1, file2, labels, ylabel, title, suptitle=None):
    df1 = pd.read_csv(os.path.join(data_dir, file1 + '.csv'), header=None)
    df2 = pd.read_csv(os.path.join(data_dir, file2 + '.csv'), header=None)
    time = df1.index / sampling_rate

    num_signals = len(labels)

    # Create 2-row subplot: top row for signal plots, bottom for errors
    fig, axs = plt.subplots(2, num_signals, figsize=(5 * num_signals, 10))

    # Handle single-column case: axs becomes 2D array always
    if num_signals == 1:
        axs = np.array([[axs[0]], [axs[1]]])

    # First row: signal plots
    for i in range(num_signals):
        axs[0, i].plot(time, df1[i], label=f'{labels[i]}')
        axs[0, i].plot(time, df2[i], label=f'{labels[i]} Desired', linestyle='--')
        axs[0, i].set_title(f'{labels[i]} vs Desired')
        axs[0, i].set_xlabel('Time (s)')
        axs[0, i].set_ylabel(ylabel)
        axs[0, i].grid(True)
        axs[0, i].legend()

    # Compute errors
    error_magnitude = np.linalg.norm(df1.values - df2.values, axis=1)
    cumulative_error = np.cumsum(error_magnitude)

    # Second row: error magnitude and cumulative error
    # Fill only the first two columns of the second row
    axs[1, 0].plot(time, error_magnitude, color='red')
    axs[1, 0].set_title('Error Magnitude')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Euclidean Distance')
    axs[1, 0].grid(True)

    axs[1, 1].plot(time, cumulative_error, color='purple')
    axs[1, 1].set_title('Cumulative Error')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Cumulative Distance')
    axs[1, 1].grid(True)

    # Hide unused axes in second row if num_signals > 2
    for j in range(2, num_signals):
        fig.delaxes(axs[1, j])

    fig.tight_layout(pad=4.0)
    if suptitle:
        fig.suptitle(suptitle, fontsize=16, y=1.02)
        fig.subplots_adjust(top=0.92)

    plt.savefig(os.path.join(output_dir, f'{file1}.png'), bbox_inches='tight')
    plt.close()


# Special plotter for torques
def plot_torques(file1, file2, file3, labels):
    df1 = pd.read_csv(os.path.join(data_dir, file1 + '.csv'), header=None)
    df2 = pd.read_csv(os.path.join(data_dir, file2 + '.csv'), header=None)
    df3 = pd.read_csv(os.path.join(data_dir, file3 + '.csv'), header=None)
    time = df1.index / sampling_rate

    fig, axs = plt.subplots(3, 3, figsize=(15, 15))
    axs = axs.flatten()

    for i in range(len(labels)):
        axs[i].plot(time, df1[i], label=f'{labels[i]} Commanded')
        axs[i].plot(time, df2[i], label=f'{labels[i]} Observed (no gravity)')
        axs[i].plot(time, df3[i], label=f'{labels[i]} Friction comp')
        axs[i].set_title(labels[i])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('N·m')
        axs[i].grid(True)
        axs[i].legend()

    fig.tight_layout(pad=4.0, w_pad=4.0)
    fig.suptitle('Plot for torques', fontsize=16, y=1.05)
    plt.savefig(os.path.join(output_dir, 'torques.png'), bbox_inches='tight')
    plt.close()


# Special plotter for joints
def plot_joints(file, labels, units):
    df1 = pd.read_csv(os.path.join(data_dir, file + '.csv'), header=None)
    time = df1.index / sampling_rate

    fig, axs = plt.subplots(3, 3, figsize=(15, 15))
    axs = axs.flatten()

    for i in range(len(labels)):
        axs[i].plot(time, df1[i], label=labels[i])
        axs[i].set_title(labels[i])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel(units)
        axs[i].grid(True)
        axs[i].legend()

        for axis in [axs[i].xaxis, axs[i].yaxis]:
            formatter = ScalarFormatter(useOffset=False, useMathText=False)
            formatter.set_scientific(False)
            axis.set_major_formatter(formatter)

    fig.tight_layout(pad=4.0, w_pad=4.0)
    fig.suptitle('Plot for Joint Accels', fontsize=16, y=1.05)
    plt.savefig(os.path.join(output_dir, f'{file}.png'), bbox_inches='tight')
    plt.close()


# === Call Plotting Functions ===

plot_data('actual_wrench', ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], 'N', 6)
plot_data('velocity', ['X', 'Y', 'Z'], 'M/S', 3)
plot_data('desired_accel', ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], 'M/S²', 6)
plot_data('accel', ['X', 'Y', 'Z'], 'M/S²', 3)
plot_data_with_error_from_zero('orientation_error', ['X', 'Y', 'Z'], 'Orientation Error (degrees)', 3)

plot_comparison(
    'translation', 'translation_d', ['X', 'Y', 'Z'], 'Distance (m)', 'translation', suptitle='Plot for translation'
)
plot_torques('torques_d', 'torques_g', 'torques_f', [f'Joint {i+1}' for i in range(7)])
plot_joints('joints_accel_d', [f'Joint {i+1} Desired Accel' for i in range(7)], 'R/S^2')
plot_joints('joints_vel', [f'Joint {i+1}' for i in range(7)], 'R/S')
