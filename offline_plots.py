import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt

# Argument parsing
parser = argparse.ArgumentParser(description='Plot data from CSV files.')
parser.add_argument('data_dir', help='Path to the data directory (containing CSVs)')
args = parser.parse_args()

data_dir = args.data_dir

# Constants
output_dir = os.path.join(data_dir, 'plots')
os.makedirs(output_dir, exist_ok=True)
sampling_rate = 50


# Generic single file plotter
def plot_data(file_name, labels, ylabel, num_columns):
    df = pd.read_csv(os.path.join(data_dir, file_name + '.csv'), header=None)
    time = df.index / sampling_rate

    plt.figure()
    for i in range(num_columns):
        plt.plot(time, df[i], label=labels[i])
    plt.title(f'Plot for {file_name}')
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f'{file_name}.png'))
    plt.close()


# Paired comparison plotter (e.g. translation vs desired)
def plot_comparison(file1, file2, labels, ylabel, title, suptitle=None):
    df1 = pd.read_csv(os.path.join(data_dir, file1 + '.csv'), header=None)
    df2 = pd.read_csv(os.path.join(data_dir, file2 + '.csv'), header=None)
    time = df1.index / sampling_rate

    fig, axs = plt.subplots(1, len(labels), figsize=(5 * len(labels), 5))
    if len(labels) == 1:
        axs = [axs]

    for i, ax in enumerate(axs):
        ax.plot(time, df1[i], label=f'{labels[i]}')
        ax.plot(time, df2[i], label=f'{labels[i]} Desired', linestyle='--')
        ax.set_title(f'{labels[i]} vs Desired')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend()

    fig.tight_layout(pad=4.0, w_pad=4.0)
    if suptitle:
        fig.suptitle(suptitle, fontsize=16, y=1.05)
    plt.savefig(os.path.join(output_dir, f'{file1}.png'), bbox_inches='tight')
    plt.close()


# Special plotter for torques
def plot_torques(file1, file2, labels):
    df1 = pd.read_csv(os.path.join(data_dir, file1 + '.csv'), header=None)
    df2 = pd.read_csv(os.path.join(data_dir, file2 + '.csv'), header=None)
    time = df1.index / sampling_rate

    fig, axs = plt.subplots(3, 3, figsize=(15, 15))
    axs = axs.flatten()

    for i in range(len(labels)):
        axs[i].plot(time, df1[i], label=f'{labels[i]} Commanded')
        axs[i].plot(time, df2[i], label=f'{labels[i]} Observed (no gravity)')
        axs[i].set_title(labels[i])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('N·m')
        axs[i].grid(True)
        axs[i].legend()

    fig.tight_layout(pad=4.0, w_pad=4.0)
    fig.suptitle('Plot for torques', fontsize=16, y=1.05)
    plt.savefig(os.path.join(output_dir, 'torques.png'), bbox_inches='tight')
    plt.close()


# === Call Plotting Functions ===

plot_data('actual_wrench', ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], 'N', 6)
plot_data('velocity', ['X', 'Y', 'Z'], 'M/S', 3)
plot_data('desired_accel', ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], 'M/S²', 6)

plot_comparison('translation', 'translation_d', ['X', 'Y', 'Z'], 'M', 'translation', suptitle='Plot for translation')
plot_torques('torques_d', 'torques_g', [f'Joint {i+1}' for i in range(7)])
