#ifndef DHT22_DATA_H
#define DHT22_DATA_H

#define GPIO_CHIP "/dev/gpiochip0"
#define DHT22_PIN_DATA_QUERY 22
#define DHT22_PIN_DATA_INPUT 26

// Forward declaration of struct and enum
enum gpiod_line_value;
enum gpiod_line_direction;
struct gpiod_line_request;

int init_dht_gpio();
int set_direction_dht_gpio(enum gpiod_line_direction direction);
int set_value_dht_gpio(enum gpiod_line_value value);
int set_interrupt_dht_gpio(uint8_t active);
int wait_interrupt_dht_gpio();

#endif
