import time
import struct
from collections import namedtuple
import errno
import os
import subprocess
import sys

def read_bmp280_pipe():
    # Run sensor reading binary - Non-blocking
    print("\n-- Running BMP280 binary --")    
    res = subprocess.Popen(["./bin/bmp280"])
    
    # Try (and retry if needed) to read from the PIPE
    raw_data = async_listen_pipe()
    
    print(f"-- Received data size: {len(raw_data)} -- ")
    for r in raw_data:
        print(hex(r), end = " ")
    print("\n")
        
    # Remove SOF and EOF characters
    payload_data = raw_data[1:-1]    
        
    # Process payload     
    BMP280_Data_t = namedtuple('DHT22_Data_t', 'temperature pressure validity')
    bmp280_data = BMP280_Data_t._make(struct.unpack("<ffbxxx", payload_data))

    print(f"Pressure: {bmp280_data.pressure} kPa")
    print(f"Temperature: {bmp280_data.temperature} C")
    
def async_listen_pipe():
    print("\n-- Listening from BMP280 pipe --")
    fifo_path = "/tmp/bmp280_pipe"
    
    for _ in range(3):
        try:
            fd = os.open(fifo_path, os.O_RDONLY)
            with os.fdopen(fd, "rb") as fifo:
                try:
                    line = fifo.readline()
                    if line:
                        print(f"-- Read successful --")
                        return line.strip()                        
                    else:  # EOF reached if writer closed the pipe
                        print("-- Writer closed the FIFO. --")
                        break
                except IOError as e:
                    if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
                        print("-- No data available, retrying... --")
                        time.sleep(0.01)
                    else:
                        raise
        except FileNotFoundError:
                print(f"-- FIFO '{fifo_path}' not found or not created yet. Waiting... -- ")
                time.sleep(0.01)
    print("-- Unable to read from pipe. --")

if __name__ == "__main__":
    sys.stdout.flush()
    read_bmp280_pipe()