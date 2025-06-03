import numpy as np
import matplotlib.pyplot as plt

# Parameters
x0 = 0.3  # Initial displacement
v0 = 0.0  # Initial velocity
m = 1.0  # Mass
k = 20.0  # Spring constant
c = 0.0  # Damping coefficient

# Time setup
dt = 0.001  # Time step
T = 10  # Total simulation time
n_steps = int(T / dt)

# Arrays to store results
t = np.linspace(0, T, n_steps)
x = np.zeros(n_steps)
v = np.zeros(n_steps)
a = np.zeros(n_steps)

# Initial conditions
x[0] = x0
v[0] = v0
a[0] = (-c * v[0] - k * x[0]) / m

# Time-stepping loop (Euler integration)
# Pre-allocate arrays
x = np.zeros(n_steps)
v = np.zeros(n_steps)
t = np.zeros(n_steps)

# Initial conditions
x[0] = x0
v[0] = v0

for i in range(1, n_steps):
    # Time step
    t_i = t[i - 1]

    # Current state
    x_i = x[i - 1]
    v_i = v[i - 1]

    # Define the derivatives
    def dx_dt(x, v):
        return v

    def dv_dt(x, v):
        return (-c * v - k * x) / m

    # RK4 steps
    k1_x = dx_dt(x_i, v_i)
    k1_v = dv_dt(x_i, v_i)

    k2_x = dx_dt(x_i + 0.5 * dt * k1_x, v_i + 0.5 * dt * k1_v)
    k2_v = dv_dt(x_i + 0.5 * dt * k1_x, v_i + 0.5 * dt * k1_v)

    k3_x = dx_dt(x_i + 0.5 * dt * k2_x, v_i + 0.5 * dt * k2_v)
    k3_v = dv_dt(x_i + 0.5 * dt * k2_x, v_i + 0.5 * dt * k2_v)

    k4_x = dx_dt(x_i + dt * k3_x, v_i + dt * k3_v)
    k4_v = dv_dt(x_i + dt * k3_x, v_i + dt * k3_v)

    # Update next values
    x[i] = x_i + (dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
    v[i] = v_i + (dt / 6) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)

    # Optional: update time array
    t[i] = t_i + dt

# Plot
plt.figure(figsize=(10, 4))
plt.plot(t, x, label='x(t) simulated')
plt.title('Spring-Damper System via Euler Integration')
plt.xlabel('Time [s]')
plt.ylabel('Displacement [m]')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
