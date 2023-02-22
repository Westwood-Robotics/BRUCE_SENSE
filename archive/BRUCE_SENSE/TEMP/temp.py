import serial
import time

port = 'COM7'
baudrate = 115200
ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)
t_bus_init = time.time()
ser.reset_input_buffer()
incoming_bytes = [0, 0]
while True:
    if ser.inWaiting() > 1:
        # Check for start of packet
        incoming_bytes.extend(ser.read(1))
        incoming_bytes.pop(0)
        if ((incoming_bytes[0] << 8) | incoming_bytes[1]) == 0xFFFF:
            # This is the beginning of the packet
            incoming_bytes.extend(ser.read(1))
            pktlen = incoming_bytes[2]
            while ser.inWaiting() < pktlen:
                if time.time() - t_bus_init > 0.02:
                    print("[PySense | WARNING] :: Status response timed out. Is dumper alright?")
                    t_bus_init = time.time()
            incoming_bytes.extend(ser.read(pktlen))
            print(incoming_bytes)
            incoming_bytes = [0, 0]
        else:
            print(incoming_bytes)



# def __char_to_hex(self, val):
#     val[0]-