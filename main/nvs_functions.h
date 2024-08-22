#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "onewire_bus.h"
#include "esp_log.h"

static const char *NTAG = "ESP_ZB_NVS";

const char *ep_to_key[4] = {"ep0", "ep1", "ep2", "ep3"};

void init_flash()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t get_address_for_ep(onewire_device_address_t *ep_addr, uint8_t *end_point)
{
    ESP_LOGI(NTAG, "Ep %d", *end_point);
    const char *ep_key = ep_to_key[*end_point];
    ESP_LOGI(NTAG, "Getting %s", ep_key);
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("onewire_storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint64_t out = 0;

        err = nvs_get_u64(nvs_handle, ep_key, &out);
        if (err == ESP_OK)
        {
            ESP_LOGI(NTAG, "%s:address: %016llX ", ep_key, out);
            memcpy(ep_addr, &out, 8);

            // ep_addr = (onewire_device_address_t)out;
        }
        else
        {
            ESP_LOGE(NTAG, "Error (%s) reading!", esp_err_to_name(err));
        }
    }
    nvs_close(nvs_handle);
    return err;
}

esp_err_t set_address_for_ep(onewire_device_address_t *ep_addr, uint8_t *end_point)
{
    ESP_LOGI(NTAG, "Ep %d", *end_point);

    const char *ep_key = ep_to_key[*end_point];
    ESP_LOGI(NTAG, "Setting %s", ep_key);
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("onewire_storage", NVS_READWRITE, &nvs_handle);
    ESP_LOGI(NTAG, "Opened Store");
    if (err != ESP_OK)
    {
        ESP_LOGE(NTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint64_t out = 0;
        //(uintptr_t)ep_addr;
        memcpy(&out, ep_addr, 8);

        err = nvs_set_u64(nvs_handle, ep_key, out);
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    ESP_LOGI(NTAG, "Closed Store");

    return err;
}
