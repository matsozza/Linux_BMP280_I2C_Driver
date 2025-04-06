#ifndef DHT22_DATA_H
#define DHT22_DATA_H

#define GPIO_CHIP "/dev/gpiochip0"
#define DHT22_PIN_DATA_QUERY 22
#define DHT22_PIN_DATA_INPUT 26

typedef struct
{
    uint16_t temperature;
    uint16_t humidity;
    uint8_t CRC;
    uint8_t validity;
} DHT22_data_t;

void init_DHT22_sensor();
DHT22_data_t query_DHT22_sensor();

#endif
