
import serial
import struct
from cobs import cobs
import subprocess

PORT = "COM8" # serial port name = arduino uno port = usb ttl converter 
BAUD_RATE = 9600

try:
    ser = serial.Serial(PORT, BAUD_RATE) # communication with STM32
    print(f"{PORT} port successfully opened. Listening for data.")
except Exception as e:
    print(f"Port couldn't be opened. Error: {e}")
    exit()


PACKAGE_FORMAT = "<IBBBBHHHHBBBB"
# < = little-endian = lsb to msb because stm32 uses like that 
# B = uint8_t = 1 byte unsigned char
# H = uint16_t = 2 byte unsigned short
# I = uint32_t = 4 byte unsigned int


def calculate_checksum(telemetry_data): # for comparing recieved and calculated checksum to detect any mismatch

    checksum = 0

    for byte in telemetry_data:
        checksum ^= byte # xoring all to calculate checksum

    return checksum


while True:
    try:
        
        rx_data= ser.read_until(b'\x00') # read until 0x00 is received
        
        if rx_data: # if not null

            encoded_package = rx_data[:-1] # deleteing the 0x00 from the received data bc cobs can't decode it, its the endpoint

            try:
                original_package = cobs.decode(encoded_package) # reverse cobs algorithm to decode the received data, decode the encoded one

            except cobs.DecodeError as e:
                print(f"Failed to decode the package: {e}")
                continue  # pass failed packeage


            recieved_checksum = original_package[-1] # last byte is checksum
            calculated_checksum = calculate_checksum(original_package[:-1]) # calculate checksum of the data except the last checksum byte

            if recieved_checksum != calculated_checksum:
                print("Received and calculated checksum doesn't match, failed package.")
                continue  # skip this package if checksum doesn't match
            
            # reverse the typecasting to turn the byte array into sepertae variables, open the struct package
            data = struct.unpack(PACKAGE_FORMAT, original_package)
            
            # data variables 
            package_no = data[0]
            
            hour = data[1]
            minute = data[2]
            second = data[3]
            
            angle = data[4]
            distance = data[5]
            
            water_level = data[6]
            soil_humidity = data[7]
            light_intensity = data[8]
            
            temperature = data[9]
            air_humidity = data[10]
            lock_flag = data[11]
            checksum = data[12]
            
            print(f"[{hour:02d}:{minute:02d}:{second:02d}] angle: {angle}° , distance: {distance} , water level: {water_level} , lock flag: {lock_flag}")
            
            
            
            if lock_flag == 1:
                print("Lock flag detected. Locking the system.")
                
                subprocess.run("rundll32.exe user32.dll,LockWorkStation", check=True)
                
                break # to prevent continuous locking


    except Exception as e:
        print(f"Error occurred while reading data or decoding it: {e}")

