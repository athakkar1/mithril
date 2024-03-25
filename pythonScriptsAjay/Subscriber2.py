import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt

# Define the angles
theta = np.linspace(0, 2*np.pi, 1000)

# Define the wavelength
wavelength = 0.1666666  # replace with your value

# Define the separation distance
d = 0.5 * wavelength

# Initialize phases
phase1 = 0.0
phase2 = 0.0

# Function to update phases when a new message is received
def on_message(client, userdata, msg):
    global phase1, phase2
    # Assuming the message payload contains the new phases as a string
    new_phases = msg.payload.decode('utf-8').split(',')
    phase1, phase2 = map(float, new_phases)

# Create an MQTT client
client = mqtt.Client()
# Set the callback function for when a new message is received
client.on_message = on_message
# Connect to the MQTT broker
client.connect("localhost", 1883, 60)
# Subscribe to the "beam" topic
client.subscribe("Beam")
# Start the MQTT loop in a separate thread
client.loop_start()

# Function to update the plot
def update(frame):
    global phase1, phase2
    r1 = np.cos(2 * np.pi * d / wavelength * np.cos(theta) + phase1)
    r2 = np.cos(2 * np.pi * d / wavelength * np.cos(theta - np.pi) + phase2)
    r = r1 + r2
    max_r_limit = 1.5  # Adjust this value as needed
    ax.set_rmax(max_r_limit)
    line.set_ydata(r)
    return line,

# Create a polar plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
line, = ax.plot(theta, np.zeros_like(theta))

# Add labels
plt.title("Radiation Pattern of Two Isotropic Antennas")

# Create an animation
ani = FuncAnimation(fig, update, frames=None, interval=200)  # Adjust interval as needed

# Show the plot
plt.show()
