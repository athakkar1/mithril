import paho.mqtt.client as mqtt
import time
import serial

broker_address = "192.168.199.1"
broker_port = 1883
topic = "test"
data_file_path = "data.txt"
serial_file_path = "/dev/ttyACM0"

def read_from_serial(serial_path):
   data = ''
   #take note of baud rate in the second input to Serial
   serial_port = serial.Serial(serial_path, 921600, timeout = 1)
   while True:
      while serial_port.in_waiting > 0:
         data = serial_port.readline().decode('utf-8', errors = 'ignore').strip()
         return data

def read_data_from_file(file_path):
   with open(file_path, 'r') as file:
      return file.read().strip()

# Create MQTT client
client = mqtt.Client()

# Connect to the broker
client.connect(broker_address, broker_port, 60)

# Publish a message every 5 seconds
try:
    while True:
       #message = read_data_from_file(data_file_path)
       message2 = read_from_serial(serial_file_path)
       client.publish(topic, message2)
       print(f"Published: {message2}")
except KeyboardInterrupt:
   # Disconnect on keyboard interrupt
   client.disconnect()
   print("Disconnected from the MQTT broker")

