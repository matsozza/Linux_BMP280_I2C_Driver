#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "gpiod.h"
#include "dht22_data.h"

struct gpiod_chip *chip;
struct gpiod_line_settings *lineSettingsQuery, *lineSettingsData;
struct gpiod_line_config *lineConfigQuery, *lineConfigData;
struct gpiod_request_config *requestConfigQuery, *requestConfigData;
struct gpiod_line_request *requestQuery, *requestData;
unsigned int *offsetsQuery, *offsetsData;

int init_dht_gpio()
{    
    // Start pointer to control offsets memory
    offsetsQuery = malloc(sizeof(unsigned int));
    offsetsData = malloc(sizeof(unsigned int));
    if (offsetsQuery == NULL) return EXIT_FAILURE;
    if (offsetsData == NULL) return EXIT_FAILURE;
    unsigned int gpio_pins_query[1] = {(unsigned int)DHT22_PIN_DATA_QUERY};
    unsigned int gpio_pins_data[1] = {(unsigned int)DHT22_PIN_DATA_INPUT};
    memcpy(offsetsQuery, gpio_pins_query, 1 * sizeof(unsigned int));
    memcpy(offsetsData, gpio_pins_data, 1 * sizeof(unsigned int));
    
    // Open GPIO chip
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) return EXIT_FAILURE;

    // Create line settings
    lineSettingsQuery = gpiod_line_settings_new();
    lineSettingsData = gpiod_line_settings_new();
    if (!lineSettingsQuery) goto ERROR;
    if (!lineSettingsData) goto ERROR;

    // Set as output with initial value as one
    gpiod_line_settings_set_direction(lineSettingsQuery, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(lineSettingsQuery, 1);
    gpiod_line_settings_set_drive(lineSettingsQuery, GPIOD_LINE_DRIVE_OPEN_DRAIN);

    gpiod_line_settings_set_direction(lineSettingsData, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_bias(lineSettingsData, GPIOD_LINE_BIAS_PULL_UP);
    //gpiod_line_settings_set_drive(lineSettingsData, GPIOD_LINE_DRIVE_OPEN_DRAIN);

    // Create new line configuration
    lineConfigQuery = gpiod_line_config_new();
    lineConfigData = gpiod_line_config_new();
    if (!lineConfigQuery) goto ERROR;
    if (!lineConfigData) goto ERROR;

    // Add configuration for selected pins as offsets
    gpiod_line_config_add_line_settings(lineConfigQuery, offsetsQuery, 1, lineSettingsQuery);
    gpiod_line_config_add_line_settings(lineConfigData, offsetsData, 1, lineSettingsData);

    // Create request config
    requestConfigQuery = gpiod_request_config_new();
    requestConfigData = gpiod_request_config_new();
    if (!requestConfigQuery) goto ERROR;
    if (!requestConfigData) goto ERROR;

    gpiod_request_config_set_consumer(requestConfigQuery, "DHT-QUERY");
    gpiod_request_config_set_consumer(requestConfigData, "DHT-DATA");

    // Get request object for GPIO line control
    requestQuery = gpiod_chip_request_lines(chip, requestConfigQuery, lineConfigQuery);
    requestData = gpiod_chip_request_lines(chip, requestConfigData, lineConfigData);
    if (!requestQuery) goto ERROR;
    if (!requestData) goto ERROR;

    return EXIT_SUCCESS;

ERROR:
    gpiod_request_config_free(requestConfigQuery);
    gpiod_line_config_free(lineConfigQuery);
    gpiod_line_settings_free(lineSettingsQuery);
    gpiod_chip_close(chip);
    return EXIT_FAILURE;
}

int set_direction_dht_gpio(enum gpiod_line_direction direction)
{   
    if(direction==GPIOD_LINE_DIRECTION_INPUT)
    {
        gpiod_line_settings_set_direction(lineSettingsQuery, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(lineSettingsQuery, GPIOD_LINE_BIAS_PULL_UP);

    }
    else if(direction==GPIOD_LINE_DIRECTION_OUTPUT)
    {
        gpiod_line_settings_set_direction(lineSettingsQuery, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(lineSettingsQuery, 1);
    }
    else
    {
        return EXIT_FAILURE;
    }

    gpiod_line_config_add_line_settings(lineConfigQuery, offsetsQuery, 1, lineSettingsQuery);
    gpiod_line_request_reconfigure_lines(requestQuery, lineConfigQuery);

    return EXIT_SUCCESS;
}

int set_interrupt_dht_gpio(uint8_t active)
{
    if(active)
    {
        gpiod_line_settings_set_edge_detection(lineSettingsData, GPIOD_LINE_EDGE_BOTH);        
    }
    else
    {
        gpiod_line_settings_set_edge_detection(lineSettingsData, GPIOD_LINE_EDGE_NONE); 
    }

    gpiod_line_config_add_line_settings(lineConfigData, offsetsData, 1, lineSettingsData);
    gpiod_line_request_reconfigure_lines(requestData, lineConfigData);

    return EXIT_SUCCESS;
}

int wait_interrupt_dht_gpio()
{
    return gpiod_line_request_wait_edge_events(requestQuery, 500*1000*1000*10);
}

int set_value_dht_gpio(enum gpiod_line_value value)
{
    return gpiod_line_request_set_values(requestQuery, (enum gpiod_line_value[1]){value});
}
