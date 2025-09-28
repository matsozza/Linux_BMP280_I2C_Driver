import time
import struct
from collections import namedtuple
import errno
import os
import subprocess
import sys
import logging 

BMP280_Data_t = namedtuple('DHT22_Data_t', 'temperature pressure validity')

# ==============================
# Configuration
# ==============================
LOG_FILE = "/tmp/bmp280.log"

# ==============================
# Setup Logging
# ==============================
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.ERROR,
    format="%(asctime)s %(levelname)s: %(message)s"
)
logger = logging.getLogger("BMP280")

def read_bmp280_pipe():
    """Reads data from BMP280 sensor via named pipe and decodes it.

    Args:
        None

    Returns:
        None
    """
    
    logger.info("\n-- Running BMP280 binary --")    
    
    # Run sensor binary in a subprocess
    res = subprocess.Popen(["./bmp280/bin/bmp280"])    
    
    # Listen to the sensor data through its dedicated pipe (async)
    raw_data = async_listen_pipe()
    
    # Confirm received data
    logger.info(f"-- Received data size: {len(raw_data)} -- ")
    for r in raw_data:
        logger.info(hex(r), end = " ")
    logger.info("\n")
        
    # Remove SOF and EOF characters
    payload_data = raw_data[1:-1]    
    
    # Unpack data        
    bmp280_data = BMP280_Data_t._make(struct.unpack("<ffbxxx", payload_data))

    # logger.info results
    logger.info(f"Pressure: {bmp280_data.pressure} kPa")
    logger.info(f"Temperature: {bmp280_data.temperature:.2f} °C")
    
    return bmp280_data


def async_listen_pipe():
    """Attempts to read binary data from BMP280 named pipe asynchronously.

    Args:
        None

    Returns:
        line (bytes): Raw binary data from the pipe (if available)
    """
    FIFO_PATH = "/tmp/bmp280_pipe"
    WAIT_TIMEOUT = 5 #sec
    POLLING_INTVL = 0.01 #sec
    
    logger.info("\n-- Listening from BMP280 pipe --")
    for _ in range(int(WAIT_TIMEOUT/POLLING_INTVL)):
        try:
            fd = os.open(FIFO_PATH, os.O_RDONLY)
            with os.fdopen(fd, "rb") as fifo:
                try:
                    line = fifo.readline()
                    if line:
                        logger.info(f"-- Read successful --")
                        return line.strip()                        
                    else:
                        logger.info("-- Writer closed the FIFO. --")
                        break
                except IOError as e:
                    if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
                        logger.error("-- No data available, retrying... --")
                        time.sleep(POLLING_INTVL)
                    else:
                        raise
        except FileNotFoundError:
                logger.error(f"-- FIFO '{FIFO_PATH}' not found or not created yet. Waiting... -- ")
                time.sleep(POLLING_INTVL)
    logger.error("-- Unable to read from pipe. --")
    return -1


if __name__ == "__main__":
    """Main entry point to execute BMP280 sensor read and decode.

    Args:
        None

    Returns:
        None
    """
    sys.stdout.flush()
    read_bmp280_pipe()
