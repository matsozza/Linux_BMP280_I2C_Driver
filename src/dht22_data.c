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

static int _init_dht_gpio();
static int _set_value_query_pin(enum gpiod_line_value value);
static int _set_interrupt_data_pin(uint8_t active);

static int _init_dht_gpio()
{
    // Start pointer to control offsets memory
    offsetsQuery = malloc(sizeof(unsigned int));
    offsetsData = malloc(sizeof(unsigned int));
    if (offsetsQuery == NULL)
        return EXIT_FAILURE;
    if (offsetsData == NULL)
        return EXIT_FAILURE;
    unsigned int gpio_pins_query[1] = {(unsigned int)DHT22_PIN_DATA_QUERY};
    unsigned int gpio_pins_data[1] = {(unsigned int)DHT22_PIN_DATA_INPUT};
    memcpy(offsetsQuery, gpio_pins_query, 1 * sizeof(unsigned int));
    memcpy(offsetsData, gpio_pins_data, 1 * sizeof(unsigned int));

    // Open GPIO chip
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip)
        return EXIT_FAILURE;

    // Create line settings
    lineSettingsQuery = gpiod_line_settings_new();
    lineSettingsData = gpiod_line_settings_new();
    if (!lineSettingsQuery)
        goto ERROR;
    if (!lineSettingsData)
        goto ERROR;

    // Set as output with initial value as one
    gpiod_line_settings_set_direction(lineSettingsQuery, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(lineSettingsQuery, 1);
    gpiod_line_settings_set_drive(lineSettingsQuery, GPIOD_LINE_DRIVE_OPEN_DRAIN);

    gpiod_line_settings_set_direction(lineSettingsData, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_bias(lineSettingsData, GPIOD_LINE_BIAS_PULL_UP);
    // gpiod_line_settings_set_drive(lineSettingsData, GPIOD_LINE_DRIVE_OPEN_DRAIN);

    // Create new line configuration
    lineConfigQuery = gpiod_line_config_new();
    lineConfigData = gpiod_line_config_new();
    if (!lineConfigQuery)
        goto ERROR;
    if (!lineConfigData)
        goto ERROR;

    // Add configuration for selected pins as offsets
    gpiod_line_config_add_line_settings(lineConfigQuery, offsetsQuery, 1, lineSettingsQuery);
    gpiod_line_config_add_line_settings(lineConfigData, offsetsData, 1, lineSettingsData);

    // Create request config
    requestConfigQuery = gpiod_request_config_new();
    requestConfigData = gpiod_request_config_new();
    if (!requestConfigQuery)
        goto ERROR;
    if (!requestConfigData)
        goto ERROR;

    gpiod_request_config_set_consumer(requestConfigQuery, "DHT-QUERY");
    gpiod_request_config_set_consumer(requestConfigData, "DHT-DATA");

    // Get request object for GPIO line control
    requestQuery = gpiod_chip_request_lines(chip, requestConfigQuery, lineConfigQuery);
    requestData = gpiod_chip_request_lines(chip, requestConfigData, lineConfigData);
    if (!requestQuery)
        goto ERROR;
    if (!requestData)
        goto ERROR;

    return EXIT_SUCCESS;

ERROR:
    gpiod_request_config_free(requestConfigQuery);
    gpiod_line_config_free(lineConfigQuery);
    gpiod_line_settings_free(lineSettingsQuery);
    gpiod_chip_close(chip);
    return EXIT_FAILURE;
}

int _set_interrupt_data_pin(uint8_t active)
{
    if (active)
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

int _set_value_query_pin(enum gpiod_line_value value)
{
    return gpiod_line_request_set_values(requestQuery, (enum gpiod_line_value[1]){value});
}

void init_DHT22_sensor()
{
    _init_dht_gpio();
    _set_interrupt_data_pin(1);
}

// DHT 22 Temperature and Humidity Sensor
DHT22_data_t query_DHT22_sensor()
{

    uint64_t t0 = 0;
    uint16_t nIntEvents = 0, totIntEvents = 0, fcnNEvents = 0;
    uint16_t timeBuffer[86];
    struct gpiod_edgeset_direction_dht_gpio_event_buffer *eventBuffer;
    DHT22_data_t returnData;
    struct timespec queryTimerReq = {(int32_t)0, (int32_t)20 * 1000000};

    // Alocate the eventBufferfor capturing the interrupt events
    // (not necessarily handled by this module in real-time)
    eventBuffer = gpiod_edge_event_buffer_new(100);

    // Query the DHT22 by grounding data pin for 20ms
    _set_value_query_pin(GPIOD_LINE_VALUE_INACTIVE);
    nanosleep(&queryTimerReq, NULL); //20ms waiting
    _set_value_query_pin(GPIOD_LINE_VALUE_ACTIVE);

    // Capture the waveforms periods in the input pin
    while (1)
    {
        if (gpiod_line_request_wait_edge_events(requestData, 1000 * 1000 * 1000 * 1) == 1)
        {
            nIntEvents = gpiod_line_request_read_edge_events(requestData, eventBuffer, 100);
            if (nIntEvents > 0)
            {
                totIntEvents += nIntEvents;
                for (int idx = 0; idx < nIntEvents; idx++)
                {
                    int64_t t1 = gpiod_edge_event_get_timestamp_ns(gpiod_edge_event_buffer_get_event(eventBuffer, idx));
                    timeBuffer[totIntEvents - nIntEvents + idx] = (t1 - t0) / 1000;
                    t0 = t1;
                }
            }
        }
        else
        {
            //printf("\nTimeout - End of capture - Capture total of %d events.", totIntEvents);
            break;
        }
    }

    // Free the event buffer associated memory
    gpiod_edge_event_buffer_free(eventBuffer);

    // Valdate and process the results
    if (totIntEvents <= 86 || totIntEvents >= 84)
    {
        uint64_t decodedStream=0;
        // Check for the two 80us sync pulses
        uint8_t syncCntr=0, idxD;
        for(idxD=0; idxD<5; idxD++)
        {
            if(timeBuffer[idxD] > 72 && timeBuffer[idxD]<88 &&
                timeBuffer[idxD+1] > 72 && timeBuffer[idxD+1]<88)
             {
                syncCntr=1;
                idxD+=2;
                break;
             }
        }

        if (syncCntr !=1)
        {
            printf("\n No sync found");
            goto ERROR;
        }


        // OK - All events captured, streamed data seems valid
        uint8_t idxE=0;
        for(uint8_t idxS=idxD; idxS<80+idxD; idxS+=2)
        {
            // Bit 0 - 50us + 26us
            // Bit 1 - 50us + 70us
            if(timeBuffer[idxS] >= 40 && timeBuffer[idxS] <= 60 &&
                timeBuffer[idxS+1] >= 16 && timeBuffer[idxS+1] <= 36)
            {
                // Zero - No action 
            }
            else if(timeBuffer[idxS] >= 40 && timeBuffer[idxS] <= 60 &&
                    timeBuffer[idxS+1] >= 60 && timeBuffer[idxS+1] <= 80)
            {
                decodedStream = (decodedStream | (1ULL << (39 - idxE))); 
            }
            else
            {
                printf("\nWrong timing -> %d and %d on idx %d", timeBuffer[idxS], timeBuffer[idxS+1], idxS);
                goto ERROR;
            }
            idxE++;
        }
    
        returnData.humidity= (uint16_t)((decodedStream & 0xAAAAAAFFFF000000) >> (6*4));
        returnData.temperature= (int16_t)((decodedStream & 0xAAAAAA0000FFFF00) >> (2*4));
        returnData.CRC= (uint8_t)((decodedStream & 0xAAAAAA00000000FF) >> (0*4));

        uint8_t CRC_calc = (uint8_t)(
                                (uint8_t)(returnData.temperature >> 8) +
                                (uint8_t)(returnData.temperature & 0xFF) +
                                (uint8_t)(returnData.humidity >> 8) + 
                                (uint8_t)(returnData.humidity & 0xFF) 
                            );

        if(returnData.CRC != CRC_calc)
        {
            printf("\nWrong CRC ->  Received %d and Calculated %d", returnData.CRC, CRC_calc);
            goto ERROR;
        }

        returnData.validity = 1;
        return returnData;
    }
    else
    {
        printf("\nWrong no. of events: %d of 84~86 expected", totIntEvents);
    }

ERROR:
    // NOK - Missing events. Error mode
    returnData.validity=0;
    return returnData;
}