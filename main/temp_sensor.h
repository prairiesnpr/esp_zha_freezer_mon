#include "onewire_bus.h"
#include "ds18b20.h"
#include "esp_log.h"

#define ONEWIRE_BUS_GPIO 7

static const char *TTAG = "ESP_ZB_DS18B20_SENSOR";

typedef struct ds18b20_device_t {
    onewire_bus_handle_t bus;
    onewire_device_address_t addr;
    uint8_t th_user1;
    uint8_t tl_user2;
    ds18b20_resolution_t resolution;
} ds18b20_device_t;

void find_onewire(ds18b20_device_handle_t *ds18b20s, uint8_t *ds18b20_device_num)
{
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // install 1-wire bus

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TTAG, "Device iterator created, start searching...");
    do
    {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[*ds18b20_device_num]) == ESP_OK)
            {
                ESP_LOGI(TTAG, "Found a DS18B20[%d], address: %016llX", *ds18b20_device_num, next_onewire_device.address);
                *ds18b20_device_num+=1;
            }
            else
            {
                ESP_LOGI(TTAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TTAG, "Searching done, %d DS18B20 device(s) found", *ds18b20_device_num);
}

