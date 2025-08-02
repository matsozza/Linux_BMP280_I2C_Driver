// ----------------------------------------------------- Includes ------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <inttypes.h>
#include <sched.h>       // sched_param, SCHED_FIFO
#include <sys/mman.h>    // mlockall, MCL_CURRENT, MCL_FUTURE
#include <errno.h>
#include <time.h>

// ------------------------------------------------ Macros & Defines ---------------------------------------------------
#define I2C_BUS "/dev/i2c-1"
#define BMP280_ADDR 0x76       // Change to 0x77 if needed
#define PRESSURE_MSB_ADDR 0xF7 // Start of pressure data

#define T_SB 0b001
#define FILTER 0b010
#define SPI3W_EN 0b0

#define OSRS_T 0b010
#define OSRS_P 0b101
#define MODE 0b11

// ------------------------------------------------------ Typedef ------------------------------------------------------
typedef struct {
    float temperature;
    float pressure;
    uint8_t validity;
} BMP280_Data_t;

// ---------------------------------------------------- Global Vars ----------------------------------------------------
// Calibration data
uint16_t  dig_t1, dig_p1;
int16_t  dig_t2, dig_t3, dig_p2, dig_p3, 
                dig_p4, dig_p5, dig_p6, 
                dig_p7, dig_p8, dig_p9;

// ----------------------------------------------------- Prototypes ----------------------------------------------------


// ---------------------------------------------------- Functions ------------------------------------------------------
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t compensate_temperature(uint32_t rawTemperature)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((rawTemperature / 8) - ((int32_t)dig_t1 * 2));
    var1 = (var1 * ((int32_t)dig_t2)) / 2048;
    var2 = (int32_t)((rawTemperature / 16) - ((int32_t)dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)dig_t3)) / 16384;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t compensate_pressure(uint32_t rawPressure)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_p6;
    var2 = var2 + ((var1 * (int64_t)dig_p5) * 131072);
    var2 = var2 + (((int64_t)dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)dig_p3) / 256) + ((var1 * ((int64_t)dig_p2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)dig_p1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - rawPressure;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)dig_p8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)dig_p7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

// BMP280 Temperature and Pressure Sensor
BMP280_Data_t read_bmp_sensor()
{   
    int fd;
    char buf[24];
    BMP280_Data_t bmp280_data = {0, 0, 0};

    printf("\nBMP280 - Polling for Temperature and Pressure Data...\n");

    // Open I2C bus
    if ((fd = open(I2C_BUS, O_RDWR)) < 0)
    {
        perror("BMP280 - Failed to open I2C bus");
        return bmp280_data;
    }

    // Select BMP280
    if (ioctl(fd, I2C_SLAVE, BMP280_ADDR) < 0)
    {
        perror("BMP280 -Failed to select BMP280");
        close(fd);
        return bmp280_data;
    }

    // Write - Set 'ctrl_meas'
    buf[0] = 0xF4;
    buf[1] = (OSRS_T << 5) | (OSRS_P << 2) | (MODE); // Full precision + forced mode. Temp is 20 bits + Pressure is 20bit
    if (write(fd, buf, 2) != 2)
    {
        perror("BMP280 - Failed to set ctrl_meas");
        close(fd);
        return bmp280_data;
    }

    // Write - Set 'config'
    buf[0] = 0xF5;
    buf[1] = (T_SB << 5) | (FILTER << 2) | (SPI3W_EN);
    if (write(fd, buf, 2) != 2)
    {
        perror("BMP280 - Failed to set config");
        close(fd);
        return bmp280_data;
    }

    // Write - Set register pointer to 0x88 (Calib. Data)
    buf[0] = 0x88;
    if (write(fd, buf, 1) != 1)
    {
        perror("BMP280 - Failed to set register to calib. data address");
        close(fd);
        return bmp280_data;
    }

    // Read 24 bytes from 0x88 (Calib. Data)
    if (read(fd, buf, 24) != 24)
    {
        perror("BMP280 - Failed to read calib. data");
        close(fd);
        return bmp280_data;
    }

    int bufIdx = 0;
    dig_t1 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_t2 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_t3 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p1 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p2 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p3 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p4 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p5 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p6 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p7 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p8 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;
    dig_p9 = (buf[bufIdx+1] << 8) | buf[bufIdx]; bufIdx+=2;

    // Write - Set register pointer to 0xF7 (Pressure MSB)
    buf[0] = PRESSURE_MSB_ADDR;
    if (write(fd, buf, 1) != 1)
    {
        perror("BMP280 - Failed to set register address");
        close(fd);
        return bmp280_data;
    }

    // Read 5 bytes from 0xF7 (Pressure and Temperature bits)
    if (read(fd, buf, 6) != 6)
    {
        perror("BMP280 - Failed to read BMP280 data");
        close(fd);
        return bmp280_data;
    }

    uint32_t pressure_raw = ((uint32_t)buf[0] << 12) 
        | ((uint32_t) buf[1] << 4) 
        |  ((uint32_t)(buf[2]) >> 4); // Combine MSB + LSB
    uint32_t temperature_raw = ((uint32_t)buf[3] << 12) 
        | ((uint32_t)buf[4] << 4)
        | ((uint32_t)(buf[5]) >> 4); // Combine MSB + LSB;

    int32_t temperature_calib = compensate_temperature(temperature_raw);
    uint32_t pressure_calib = compensate_pressure(pressure_raw);

    printf("BMP280 - Pressure Data: 0x%X (%d) -> %f kPa\n", pressure_calib, pressure_calib, ((float)pressure_calib)/100000);
    printf("BMP280 - Temperature Data: 0x%X (%d) -> %f degC\n\n", temperature_calib, temperature_calib, ((float)temperature_calib)/100);
    bmp280_data.temperature = (float)temperature_calib/100;
    bmp280_data.pressure =  (float)pressure_calib/100000;
    bmp280_data.validity = 1;

    close(fd);

    return bmp280_data;
}

// Miliseconds sleep
int msleep(long msec) {
    struct timespec ts;
    int res;

    if (msec < 0) return -1;

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}


// Main routine for data collection
int main()
{
    /* Define pipe */
    int stt = mkfifo("/tmp/bmp280_pipe", 0666);
    if(stt == -1)
    {
        perror("BMP280 Intf - Error creating bmp280 pipe");    
        unlink("/tmp/bmp280_pipe");
        return 1;
    }
    printf("BMP280 Intf - Pipe created!\n");
    
    /* Read sensor data */
    BMP280_Data_t bmp280_data;
    bmp280_data = read_bmp_sensor();
    
    /* Insert data into pipe - Try to open pipe w/o blocking */
    int fd;
    int attemptCntr=0;
    printf("BMP280 Intf - Attempt 1 to open the bmp280 pipe");
    while((fd=open("/tmp/bmp280_pipe", O_WRONLY | O_NONBLOCK)) == -1)
    {
        printf("\rBMP280 Intf - Attempt %d to open the bmp280 pipe", attemptCntr+1);
        if (attemptCntr > 50)
        {
            perror("BMP280 Intf - Aborting attempt to open bmp280 pipe after many attempts");
            close(fd);
            unlink("/tmp/bmp280_pipe");
            return 1;
        }       
        attemptCntr++;
        msleep(10);
    }
    printf("\nBMP280 Intf - Pipe opened!\n");

    write(fd, "|", 1);
    write(fd, (const void*) &bmp280_data, sizeof(bmp280_data));
    write(fd, "*", 1);
    printf("BMP280 Intf - BMP280 info written to the pipe!\n");

    close(fd);
    unlink("/tmp/bmp280_pipe");
    return 0;
}