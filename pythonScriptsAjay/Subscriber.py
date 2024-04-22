import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

broker_address = "localhost"
broker_port = 1883
message1 = "FFT"
data_file_path = "received_data2.txt"
previous_values = False
buffer = None
current_values = np.zeros(1024)
vmin = 0
vmax = 10

# Callback when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(message1)

# Callback when a message is received from the MQTT broker
def on_message(client, userdata, msg):
    if msg.topic == "FFT":
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
    global previous_values
    global buffer
    global current_values

    # Read data from serial port
    data = penis.payload.decode()
    if data:
        # Process the data (split by commas and remove newline character)
        try:
            values = np.array([int(val) for val in data.strip().split(',')])
            '''
            if previous_values:
                values = abs(values - buffer)*10
                current_values = values
                previous_values = False
                print(values)
            else:
                buffer = values
                values = current_values
                previous_values = True
            '''
            if len(values) != num_points:
                return  # Skip if the number of data points does not match num_points
        except ValueError:
            # Handle cases where data is not a valid float
            return

        # Append new FFT data to the 2D array
        if fft_data.shape[0] > 40:
            fft_data = fft_data[1:]
        fft_data = np.vstack([fft_data, values])

        # Clear the axis and plot the spectrogram
        im.set_data(fft_data)
        

# Create MQTT client
client = mqtt.Client()

# Set callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(broker_address, broker_port, 60)
# Initialize parameters
sampling_frequency = 21929  # Updated sampling frequency
fft_size = 2048  # FFT size
num_points = 1024  # Number of points per FFT
c = 3 * (10**8)
Td = 258 * (10**-6)
b = 50 * (10**6)
range_constant = Td*c/(b*2)
# Create the plot with a constant color scale outside of the function
fig, ax = plt.subplots()

# Initialize fft_data with a row of zeros
fft_data = np.zeros((40, num_points))

# Create the plot with a constant color scale
im = ax.imshow(fft_data, aspect='auto', vmin=vmin, vmax=vmax, origin='lower', cmap='Oranges')

num_ticks = 5  # adjust this value as needed
xticks = np.linspace(1, num_points, num_ticks)
xfreqs = xfreqs = ["{:.2f}".format(x) for x in xticks * sampling_frequency * range_constant / fft_size]
ax.set_xticks(xticks)
ax.set_xticklabels(xfreqs)

# Add a colorbar
fig.colorbar(im, ax=ax)

ax.set_xlabel('Range (m)')
ax.set_ylabel('Time Index')
ax.set_title('Real-time FFT Spectrogram')

#Initililize penis
penis = 0
penis2 = 0
# Start the MQTT client loop in a non-blocking way
client.loop_start()

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(1000), interval=30, cache_frame_data=False)

# Display the plot
plt.show()

# Close the MQTT client when the plot is closed
client.loop_stop()
