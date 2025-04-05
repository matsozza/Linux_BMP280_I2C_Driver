#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include "gpiod.h"
#include "dht22_data.h"
#include <inttypes.h>

#define I2C_BUS "/dev/i2c-1"
#define BMP280_ADDR 0x76       // Change to 0x77 if needed
#define PRESSURE_MSB_ADDR 0xF7 // Start of pressure data

#define T_SB 0b001
#define FILTER 0b010
#define SPI3W_EN 0b0

#define OSRS_T 0b010
#define OSRS_P 0b101
#define MODE 0b11

extern struct gpiod_chip *chip;
extern struct gpiod_line_settings *lineSettingsQuery, *lineSettingsData;
extern struct gpiod_line_config *lineConfigQuery, *lineConfigData;
extern struct gpiod_request_config *requestConfigQuery, *requestConfigData;
extern struct gpiod_line_request *requestQuery, *requestData;
extern unsigned int *offsetsQuery, *offsetsData;

// Calibration data
uint16_t  dig_t1, dig_p1;
int16_t  dig_t2, dig_t3, dig_p2, dig_p3, 
                dig_p4, dig_p5, dig_p6, 
                dig_p7, dig_p8, dig_p9;

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
void read_sensor()
{   
    int fd;
    char buf[24];

    // Open I2C bus
    if ((fd = open(I2C_BUS, O_RDWR)) < 0)
    {
        perror("Failed to open I2C bus");
        return 1;
    }

    // Select BMP280
    if (ioctl(fd, I2C_SLAVE, BMP280_ADDR) < 0)
    {
        perror("Failed to select BMP280");
        close(fd);
        return 1;
    }

    // Write - Set 'ctrl_meas'
    buf[0] = 0xF4;
    buf[1] = (OSRS_T << 5) | (OSRS_P << 2) | (MODE); // Full precision + forced mode. Temp is 20 bits + Pressure is 20bit
    if (write(fd, buf, 2) != 2)
    {
        perror("Failed to set ctrl_meas");
        close(fd);
        return 1;
    }

    // Write - Set 'config'
    buf[0] = 0xF5;
    buf[1] = (T_SB << 5) | (FILTER << 2) | (SPI3W_EN);
    if (write(fd, buf, 2) != 2)
    {
        perror("Failed to set config");
        close(fd);
        return 1;
    }

    // Write - Set register pointer to 0x88 (Calib. Data)
    buf[0] = 0x88;
    if (write(fd, buf, 1) != 1)
    {
        perror("Failed to set register to calib. data address");
        close(fd);
        return 1;
    }

    // Read 24 bytes from 0x88 (Calib. Data)
    if (read(fd, buf, 24) != 24)
    {
        perror("Failed to read calib. data");
        close(fd);
        return 1;
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

    for (int n = 0; n < 50; n++)
    {
        // Write - Set register pointer to 0xF7 (Pressure MSB)
        buf[0] = PRESSURE_MSB_ADDR;
        if (write(fd, buf, 1) != 1)
        {
            perror("Failed to set register address");
            close(fd);
            return 1;
        }

        // Read 5 bytes from 0xF7 (Pressure and Temperature bits)
        if (read(fd, buf, 6) != 6)
        {
            perror("Failed to read BMP280 data");
            close(fd);
            return 1;
        }

        uint32_t pressure_raw = ((uint32_t)buf[0] << 12) 
            | ((uint32_t) buf[1] << 4) 
            |  ((uint32_t)(buf[2]) >> 4); // Combine MSB + LSB
        uint32_t temperature_raw = ((uint32_t)buf[3] << 12) 
            | ((uint32_t)buf[4] << 4)
            | ((uint32_t)(buf[5]) >> 4); // Combine MSB + LSB;

        int32_t temperature_calib = compensate_temperature(temperature_raw);
        uint32_t pressure_calib = compensate_pressure(pressure_raw);

        printf("\nPressure Data: 0x%X (%d) -> %f kPa\n", pressure_raw, pressure_raw, ((float)pressure_calib)/100000);
        printf("Temperature Data: 0x%X (%d) -> %f degC\n", temperature_raw, temperature_raw, ((float)temperature_calib)/100);
        sleep(1);
    }

    close(fd);
}

// DHT 22 Temperature and Humidity Sensor
void read_sensor2()
{
    init_dht_gpio();
    set_interrupt_dht_gpio(1);

    struct timespec req = {0};
    req.tv_sec = 0;        
    req.tv_nsec = 20 * 1000000; 

    printf("\nStarting!");
    for(int idxS=0; idxS<10; idxS++)
    {
        //struct timespec t0, t1;
        printf("\n------------------------------------------------------------");
        uint64_t t0=0; 
        uint16_t nEvents=0, actNEvents=0,fcnNEvents=0;

        struct gpiod_edge_event_buffer *eventBuffer;
        eventBuffer = gpiod_edge_event_buffer_new(100);
        uint16_t timeBuffer[86];

        //timespec_get(&t0, TIME_UTC);

        set_value_dht_gpio(GPIOD_LINE_VALUE_INACTIVE);
        nanosleep(&req, NULL);
        set_value_dht_gpio(GPIOD_LINE_VALUE_ACTIVE);
        //set_direction_dht_gpio(GPIOD_LINE_DIRECTION_INPUT);

        // Commute to input
        while(1)
        {
            if(gpiod_line_request_wait_edge_events(requestData, 500*1000*1000*1)==1)
            {
                nEvents = gpiod_line_request_read_edge_events(requestData, eventBuffer, 100);
                if(nEvents>0)
                {
                    //printf("\n--------------- nEvents: %d", nEvents);
                    actNEvents += nEvents;
                    //printf("\n--------------- Act nEvents: %d", actNEvents);
            
                    fcnNEvents=gpiod_edge_event_buffer_get_num_events(eventBuffer);
                    for(int idx=0; idx < fcnNEvents; idx++)
                    {
                        int64_t t1 = gpiod_edge_event_get_timestamp_ns(gpiod_edge_event_buffer_get_event(eventBuffer, idx));
                        timeBuffer[actNEvents-fcnNEvents+idx]=(t1-t0)/1000;
                        printf("\nEvent %d: %lld us", idx, (t1-t0)/1000);
                        t0=t1;
                    }
                }
            }
            else
            {
                printf("\nTimeout - Capture total of %d events.", actNEvents);
                break;
            }
            
        }
        //set_direction_dht_gpio(GPIOD_LINE_DIRECTION_OUTPUT);
        sleep(2);
    }
}

int main()
{
    read_sensor2();
}