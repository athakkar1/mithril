import numpy as np
import matplotlib.pyplot as plt

# Define the angles
theta = np.linspace(0, 2*np.pi, 1000)

# Define the wavelength
wavelength = .1666666  # replace with your value

# Define the separation distance
d = 0.5 * wavelength

# Define the phases
phase1 = 0.0  # replace with your value
phase2 = 0.0  # replace with your value

# Calculate the fields due to each antenna
r1 = np.cos(2 * np.pi * d / wavelength * np.cos(theta) + phase1)
r2 = np.cos(2 * np.pi * d / wavelength * np.cos(theta - np.pi) + phase2)

# Add the fields together
r = r1 + r2

# Create a polar plot
plt.figure()
plt.polar(theta, r)

# Add labels
plt.title("Radiation Pattern of Two Isotropic Antennas")
plt.show()