import numpy as np
import matplotlib.pyplot as plt
import tqdm
from matplotlib.animation import FuncAnimation

size = 51  # Size of the 2D grid

# Constants
# c = 343  # Speed of sound in air in m/s
# dx = 0.001  # Spatial resolution in meters
dt = 0.0001 # dx / c / 2  # Time step (CFL condition)
# dt = 0.00037037

# Domain
time_steps = 1000  # Number of time steps to simulate

# Initialize pressure and pressure derivatives
p = np.zeros((size, size))
p_prev = np.zeros((size, size))
p_next = np.zeros((size, size))

# Initial conditions: place a source
frequency = 300 # 1712  # Frequency of the source in Hz
amplitude = 0.05
damper = 0  # Damping factor

frames = []

# Simulation loop
for t in tqdm.tqdm(range(time_steps)):
    p[size//2, size//2] = amplitude * np.sin(2 * np.pi * frequency * t * dt)
    for i in range(1, size-1):
        for j in range(1, size-1):
            p_next[i, j] = 0.25 * (p[i+1, j] + p[i-1, j] + p[i, j+1] + p[i, j-1] - 4*p[i, j]) + 2*p[i, j] - p_prev[i, j]

    # Reflective boundary conditions
    p_next[0, :] = p_next[1, :] * damper
    p_next[-1, :] = p_next[-2, :] * damper
    p_next[:, 0] = p_next[:, 1] * damper
    p_next[:, -1] = p_next[:, -2] * damper

    # Update pressures for next time step
    p_prev, p, p_next = p, p_next, p_prev

    # Save the frame
    frames.append(p.copy())

# Plot the final state of the wave
fig = plt.figure()
f = lambda frame: (plt.imshow(frames[frame], vmin=-0.5, vmax=0.5),)
ani = FuncAnimation(fig, f, len(frames), interval = 10, repeat_delay=1000, repeat=True, blit=True)
plt.show()
