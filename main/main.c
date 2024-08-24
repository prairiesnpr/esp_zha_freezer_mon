#include "main.h"

#include "esp_check.h"
#include "string.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "temp_sensor.h"
#include "nvs_functions.h"


static const char *TAG = "ESP_ZB_TEMP_SENSOR";
uint8_t read_failures = 0;

uint8_t ds18b20_device_num = 0;
ds18b20_device_handle_t ds18b20s[HA_ESP_NUM_T_SENSORS];
uint8_t ep_to_ds[HA_ESP_NUM_T_SENSORS]; // Use 0xff to indicate no link

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static void start_temp_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &temp_timer_callback,
        .name = "temp_timer"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ESP_TEMP_SENSOR_UPDATE_INTERVAL));
}

static void start_heart_beat_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &heart_beat_timer_callback,
        .name = "heart_beat_timer"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ESP_BINARY_HB_UPDATE_INTERVAL));
}

void report_heart_beat_attr(uint8_t ep)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send HB 'report attributes' command");
}

void report_temp_attr(uint8_t ep)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");
}

void esp_app_temp_sensor_handler(float temperature, uint8_t endpoint)
{
    int16_t measured_value = zb_temperature_to_s16(temperature);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_lock_release();
}

void esp_app_heart_beat_handler()
{

    esp_zb_zcl_attr_t *heart_beat = esp_zb_zcl_get_attribute(
        HA_ESP_HB_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID);
    bool cur_val = *(bool *)heart_beat->data_p;

    ESP_LOGI(TAG, "Heart Beat: %s", cur_val ? "On" : "Off");

    cur_val = cur_val ^ 1;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_HB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &cur_val, false);
    esp_zb_lock_release();
}

void read_temps(float *temp_results)
{
    float temperature;
    vTaskDelay(pdMS_TO_TICKS(200));
    for (uint8_t i = 0; i < ds18b20_device_num; i++)
    {
        ESP_LOGI(TAG, "Setting Temp Conv");
        esp_err_t err = ds18b20_trigger_temperature_conversion(ds18b20s[i]);
        if (err == ESP_OK)
        {
            err = ds18b20_get_temperature(ds18b20s[i], &temperature);
            if (err == ESP_OK)
            {
                temp_results[i] = temperature;
            }
            else
            {
                temp_results[i] = DSB1820_BAD_TEMP;
                read_failures++;
            }
        }
        else
        {
            temp_results[i] = DSB1820_BAD_TEMP;
            read_failures++;
        }
    }
    if (read_failures > (4 * HA_ESP_NUM_T_SENSORS))
    {
        // Had to be a way to reset the bus, but I haven't found it.
        onewire_bus_reset(ds18b20s[0]->bus);
    }
}
static void temp_timer_callback(void *arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Temperature timer called, time since boot: %lld us", time_since_boot);
    float temp_results[HA_ESP_NUM_T_SENSORS] = {DSB1820_BAD_TEMP, DSB1820_BAD_TEMP, DSB1820_BAD_TEMP, DSB1820_BAD_TEMP};
    read_temps(temp_results);

    for (int i = 0; i < HA_ESP_NUM_T_SENSORS; i++)
    {
        uint8_t dsb_index = ep_to_ds[i];
        ESP_LOGI(TAG, "EP: %d, dsb_index: %d", i, dsb_index);

        if (dsb_index == 0xff)
        {
            ESP_LOGI(TAG, "No sensor for EP: %d", i);
        }
        else
        {
            ESP_LOGI(TAG, "temperature read from DS18B20[%d], for EP: %d,  %.2fC", dsb_index, i, temp_results[dsb_index]);
            if (temp_results[dsb_index] != DSB1820_BAD_TEMP)
            {
                esp_app_temp_sensor_handler(temp_results[dsb_index], (HA_ESP_TEMP_START_ENDPOINT + i));
                report_temp_attr(HA_ESP_TEMP_START_ENDPOINT + i);
            }
        }
    }
}

static void heart_beat_timer_callback(void *arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Heart Beat timer called, time since boot: %lld us", time_since_boot);
    esp_app_heart_beat_handler();
    report_heart_beat_attr(HA_ESP_HB_ENDPOINT);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    memset(ep_to_ds, 0xff, HA_ESP_NUM_T_SENSORS);

    find_onewire(ds18b20s, &ds18b20_device_num);
    // We have an array of all found ds18b20s
    ESP_LOGI(TAG, "One Wire Count: %i", ds18b20_device_num);

    uint8_t associated_eps_count = 0;
    // uint8_t unassociated_sensor_count = 0;
    //  For each found ds18b20, we need to try to match it to an endpoint
    for (uint8_t i = 0; i < HA_ESP_NUM_T_SENSORS; i++)
    {
        // for each endpoint see if we have a saved address
        onewire_device_address_t found_addr = 0;
        uint8_t *ep_index = &i;
        esp_err_t r_result = get_address_for_ep(&found_addr, ep_index);
        ESP_LOGI(TAG, "Err Result %d, address: %016llX", r_result, found_addr);
        if (r_result == ESP_OK && found_addr != 0)
        {
            // We have identified a saved address, but do we have a sensor that matches?
            bool dev_exists = 0x00;
            for (uint8_t j = 0; j < ds18b20_device_num; j++)
            {

                if (found_addr == ds18b20s[j]->addr)
                {
                    dev_exists = 0x01;
                    // associate dev with correct endpoint
                    ep_to_ds[j] = i;
                    ESP_LOGI(TAG, "DS18B20[%d], address: %016llX, associated with EP index: %d", j, found_addr, i);
                    associated_eps_count++;

                    break;
                }
            }

            if (!dev_exists)
            {
                // We no longer have it, so remove it from the saved state.
                ESP_LOGI(TAG, "Address: %016llX, associated with EP index: %d, deleted", found_addr, i);
                found_addr = 0;
                r_result = set_address_for_ep(&found_addr, ep_index);
            }
        }
    }

    // Ok, we have cleaned our list, now to assign extra sensors to endpoints
    // We know that anything with 0xff in ep_to_ds needs a sensor
    uint8_t available_sensors = ds18b20_device_num - associated_eps_count;
    uint8_t required_sensors = HA_ESP_NUM_T_SENSORS - associated_eps_count;

    ESP_LOGI(TAG, "There are %d sensors available to assign, need %d.", available_sensors, required_sensors);
    uint8_t av_sensor_index[available_sensors];
    memset(av_sensor_index, 0xff, available_sensors);

    uint8_t sensors_to_assign = available_sensors;
    if (available_sensors >= required_sensors)
    {
        sensors_to_assign = required_sensors;
    }

    for (uint8_t i = 0; i < sensors_to_assign; i++)
    {
        for (uint8_t j = 0; j < HA_ESP_NUM_T_SENSORS; j++)
        {
            if (ep_to_ds[j] == 0xff)
            {
                // Needs a sensor
                for (uint8_t k = 0; k < ds18b20_device_num; k++)
                {
                    bool sens_av = 0x01;
                    for (uint8_t l = 0; l < HA_ESP_NUM_T_SENSORS; l++)
                    {
                        if (ep_to_ds[l] == k)
                        {
                            sens_av = 0x00;
                            break;
                        }
                    }
                    if (sens_av)
                    {
                        ep_to_ds[j] = k;
                        uint64_t set_addr = (uint64_t)ds18b20s[k]->addr;
                        esp_err_t r_result = set_address_for_ep(&set_addr, &j);
                        if (r_result != ESP_OK)
                        {
                            ESP_LOGW(TAG, "Failed to save ep %d.", j);
                        }
                        ESP_LOGI(TAG, "Setting EP %d to Sensor index %d, address: %016llX", j, k, ds18b20s[k]->addr);
                        break;
                    }
                }
            }
        }
    }

    // Just print the final result so we can watch it over time.
    for (uint8_t i = 0; (i < HA_ESP_NUM_T_SENSORS) && (i < ds18b20_device_num); i++)
    {
        ESP_LOGI(TAG, "EP %d,  address: %016llX", i, ds18b20s[ep_to_ds[i]]->addr);
    }

    ESP_LOGI(TAG, "Starting Timers");
    start_temp_timer();
    start_heart_beat_timer();
    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(esp_zb_temperature_meas_cluster_cfg_t *temperature_sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *t_attr_list = esp_zb_temperature_meas_cluster_create(temperature_sensor);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, t_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_cluster_list_t *add_basic_and_identify_clusters_create(
    esp_zb_identify_cluster_cfg_t *identify_cluster_cfg,
    esp_zb_basic_cluster_cfg_t *basic_cluster_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(basic_cluster_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(identify_cluster_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    return cluster_list;
}

static void custom_temperature_sensor_ep_create(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, esp_zb_temperature_meas_cluster_cfg_t *temperature_sensor)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_cluster_list_t *t_cl = custom_temperature_sensor_clusters_create(temperature_sensor);
    esp_zb_ep_list_add_ep(ep_list, t_cl, endpoint_config);
}

static void binary_sensor_ep_create(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, esp_zb_cluster_list_t *binary_sensor_cluster_list)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, binary_sensor_cluster_list, endpoint_config);
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };

    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE};

    esp_zb_binary_input_cluster_cfg_t heart_beat_sensor_cfg = {
        .out_of_service = ESP_ZB_ZCL_BINARY_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE,
        .status_flags = ESP_ZB_ZCL_BINARY_INPUT_STATUS_FLAG_DEFAULT_VALUE};

    esp_zb_cluster_list_t *heart_beat_cluster_list = add_basic_and_identify_clusters_create(
        &identify_cluster_cfg,
        &basic_cluster_cfg);

    esp_zb_attribute_list_t *hb_attr_list = esp_zb_binary_input_cluster_create(&heart_beat_sensor_cfg);
    bool bin_val = 0x00;
    esp_zb_binary_input_cluster_add_attr(hb_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, HB_IDENTIFIER);
    esp_zb_binary_input_cluster_add_attr(hb_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &bin_val);

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(heart_beat_cluster_list, hb_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    binary_sensor_ep_create(ep_list, HA_ESP_HB_ENDPOINT, heart_beat_cluster_list);

    for (uint8_t ep = HA_ESP_TEMP_START_ENDPOINT;
         ep < (HA_ESP_TEMP_START_ENDPOINT + HA_ESP_NUM_T_SENSORS);
         ep++)
    {
        esp_zb_temperature_meas_cluster_cfg_t temp_sensor_cfg = {
            .max_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE),
            .min_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE),
            .measured_value = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN};

        custom_temperature_sensor_ep_create(ep_list, ep, &temp_sensor_cfg);
    }
    esp_zb_device_register(ep_list);
    for (uint8_t ep = HA_ESP_TEMP_START_ENDPOINT;
         ep < (HA_ESP_TEMP_START_ENDPOINT + HA_ESP_NUM_T_SENSORS);
         ep++)
    {
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = ep,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .dst.endpoint = 1,
            .u.send_info.min_interval = 1,
            .u.send_info.max_interval = 0,
            .u.send_info.def_min_interval = 1,
            .u.send_info.def_max_interval = 0,
            .u.send_info.delta.u16 = 100,
            .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };

        ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_info));
    }
    esp_zb_zcl_reporting_info_t hb_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_HB_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u8 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&hb_reporting_info));

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}