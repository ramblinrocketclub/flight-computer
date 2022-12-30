import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
serial_instance = serial.Serial()

port_list = []

for port in ports:
    port_list.append(str(port))
    print(str(port))

value = input("Select port: ")

for i in range(0, len(port_list)):
    if port_list[i].startswith(str(value)):
        port_variable = str(value)

serial_instance.baudrate = 4000000
serial_instance.port = port_variable
serial_instance.open()

while True:
    if serial_instance.in_waiting:
        packet = serial_instance.readline()
        print(packet.decode('utf').rstrip("\n"))
