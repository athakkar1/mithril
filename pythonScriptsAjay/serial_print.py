import serial

print(serial)

serial_port = serial.Serial('/dev/ttyACM0', 921600, timeout=1)

data = ''
while True:
  #print "----"
  while serial_port.in_waiting > 0:
    data = serial_port.readline().decode('utf-8', errors='ignore').strip()
    values = [float(val) if val.strip() else 0.0 for val in data.split(',')]
    print(values)
    print(len(values))

