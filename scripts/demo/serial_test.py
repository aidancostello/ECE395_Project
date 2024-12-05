from constants import *

import serial

ser = serial.Serial(port=PORT, baudrate=BAUD)

if not ser.isOpen():
    print("ERR: port busy or does not exist")
    exit()
    
while True:
    user_input = input(">> ")

    if user_input == 'q':
        ser.close()
        break

    input_list = user_input.split(",")

    if len(input_list) != 3:
        print("ERR: must provide 3 coordinates")
        continue

    serialized_data = bytearray([])

    for val in input_list:
        coord = 0

        try:
            coord = float(val.strip())
        except:
            print(f"ERR: \'{val}\' is not a float")
            continue

        coord = int(coord*SERIAL_SCALAR)

        for i in range(8):
            serialized_data.append(coord&0xFF)
            coord = coord >> 8
    
    ser.write(serialized_data)
    # print(serialized_data)

    





