import paho.mqtt.client as mqtt
import time
import serial
import msvcrt

broker_address = "localhost"
broker_port = 1883
topic1 = "FFT"
topic2 = "Beam"
data_file_path = "data.txt"
serial_file_path = "COM4"

def read_from_serial(serial_path):
   data = ''
   #take note of baud rate in the second input to Serial
   serial_port = serial.Serial(serial_path, 921600, timeout = 1)
   if msvcrt.kbhit():
      char = msvcrt.getwche()       
      serial_port.write(char.encode())  # send the character over the UART
   while True:
      while serial_port.in_waiting > 0:
        data = serial_port.readline().decode('utf-8', errors='ignore').strip()
        data_list = data.split(':')
        if(len(data_list) == 2):
          first_string = data_list[0]
          second_string = data_list[1]
        else:
           first_string = data_list[0]
           second_string = ""
        return first_string, second_string

def read_data_from_file(file_path):
   with open(file_path, 'r') as file:
      return file.read().strip()

# Create MQTT client
client = mqtt.Client()

# Connect to the broker
client.connect(broker_address, broker_port, 60)

previous_message = False
buffer = None

# Publish a message every 5 seconds
try:
    while True:
       #message = read_data_from_file(data_file_path)
       message1, message2 = read_from_serial(serial_file_path)
       client.publish(topic1, message1)
       client.publish(topic2, message2)
       print(f"Published: {message1}")
       print(f"Published: {message2}")
except KeyboardInterrupt:
   # Disconnect on keyboard interrupt
   client.disconnect()
   print("Disconnected from the MQTT broker")

