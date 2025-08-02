# BMP280 Sensor Interface

A lightweight, cross-platform interface to poll temperature and pressure data from the BMP280 sensor using a C binary and decode it  in Python. Developed into a Raspberry PI 3 board.

## 📦 Project Structure

This repository contains three key components:

| File                | Description                                                                 |
|---------------------|------------------------------------------------------------------------------|
| `src/bmp280.c`      | C source code for reading BMP280 data over I2C and sending it via a FIFO pipe. |
| `src/bmp280.py`     | Python script that runs the C binary, reads binary data from the pipe, and unpacks it. |
| `Makefile`          | Cross-compilation and deployment setup for building and testing on embedded targets. |

## 🚀 How It Works

1. The Makefile compiles `bmp280.c` into a static ARM64 binary.
2. The Python script runs the binary, reads the structured binary payload from `/tmp/bmp280_pipe`, and decodes temperature and pressure values.
3. Output is printed with human-readable formatting and framed with pipe start/end markers for integrity.

## 🔧 Build & Run

```bash
make           # Build binary
make test      # Compile + send to remote device and run Python script
make clean     # Clean previous build artifacts
