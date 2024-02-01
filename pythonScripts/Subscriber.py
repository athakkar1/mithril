import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

broker_address = "localhost"
broker_port = 1883
topic = "test"
data_file_path = "received_data2.txt"

# Callback when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(topic)

# Callback when a message is received from the MQTT broker
def on_message(client, userdata, msg):
    global penis
    penis = msg
    #update(msg)
    #received_message = msg.payload.decode()
    #print(f"Received message: {msg.payload.decode('utf-8')}")
    #with open(data_file_path, 'a') as file:
        #file.write(received_message + "\n")
        #print(f"Data written to {data_file_path}")

def update(frame):
    global fft_data

    # Read data from serial port
    data = penis.payload.decode()
    if data:
        # Process the data (split by commas and remove newline character)
        try:
            values = [float(val) for val in data.strip().split(',')]
          if len(values) != num_points:
                return  # Skip if the number of data points does not match num_points
        except ValueError:
            # Handle cases where data is not a valid float
            return

        # Append new FFT data to the 2D array
        fft_data = np.vstack([fft_data, values])

        # Clear the axis and plot the spectrogram
        ax.clear()
        freqs = np.arange(num_points) * sampling_frequency / fft_size
        ax.pcolormesh(freqs, np.arange(fft_data.shape[0]), fft_data, shading='gourau')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Time Index')
        ax.set_title('Real-time FFT Spectrogram')

# Create MQTT client
client = mqtt.Client()

# Set callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(broker_address, broker_port, 60)

# Create a figure for the spectrogram
fig, ax = plt.subplots()

# Initialize parameters
sampling_frequency = 46729  # Updated sampling frequency
fft_size = 2048  # FFT size
num_points = 512  # Number of points per FFT

# Initialize a 2D array to store the FFT data
fft_data = np.zeros((0, num_points))  # Changed to an empty array with correct numbe>

#Initililize penis
penis = 0

# Start the MQTT client loop in a non-blocking way
client.loop_start()

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(1000), interval=30, cache_frame_data=False)

# Display the plot
plt.show()

# Close the MQTT client when the plot is closed
client.loop_stop()
