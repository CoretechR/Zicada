// 2025 Maximilian Kern
// Based on Glen Akins' homebrew Zigbee Devices: https://github.com/bikerglen/homebrew_zigbee_devices

//---------------------------------------------------------------------------------------------
// includes
//

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <dk_buttons_and_leds.h>
#include <ram_pwrdn.h>
#include <drivers/include/nrfx_saadc.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_zcl_reporting.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include <zb_zcl_rel_humidity_measurement.h>
#include "zb_mem_config_custom.h"
#include "zb_zicada.h"

//---------------------------------------------------------------------------------------------
// defines
//

// Uncomment to enable reports on release in addition to the standard reports on press
#define ENABLE_BUTTON_RELEASE_REPORTS

// Basic cluster attributes initial values. For more information, see section 3.2.2.2 of the ZCL specification.
#define ZICADA_INIT_BASIC_APP_VERSION		01									// Version of the application software (1 byte).
#define ZICADA_INIT_BASIC_STACK_VERSION		01									// Version of the implementation of the Zigbee stack (1 byte).
#define ZICADA_INIT_BASIC_HW_VERSION		01									// Version of the hardware of the device (1 byte).
#define ZICADA_INIT_BASIC_MANUF_NAME		"kernm.de"							// Manufacturer name (32 bytes).
#define ZICADA_INIT_BASIC_MODEL_ID			"Zicada"							// Model number assigned by the manufacturer (32-bytes long string).
#define ZICADA_INIT_BASIC_DATE_CODE			"20250801"							// Date provided by the manufacturer of the device in ISO 8601 format (YYYYMMDD), for the first 8 bytes. The remaining 8 bytes are manufacturer-specific.
#define ZICADA_INIT_BASIC_POWER_SOURCE		ZB_ZCL_BASIC_POWER_SOURCE_BATTERY	// Type of power source or sources available for the device. For possible values, see section 3.2.2.2.8 of the ZCL specification.
#define ZICADA_INIT_BASIC_LOCATION_DESC		"Home"								// Description of the physical location of the device (16 bytes). You can modify it during the commisioning process.
#define ZICADA_INIT_BASIC_PH_ENV			ZB_ZCL_BASIC_ENV_UNSPECIFIED		// Description of the type of physical environment. For possible values, see section 3.2.2.2.10 of the ZCL specification.

// source endpoint for our device
#define SOURCE_ENDPOINT            1

// destination short address and endpoint (always endpoint 1 on the coordinator)
#define DEST_SHORT_ADDR            0x0000
#define DEST_ENDPOINT              1

// Do not erase NVRAM to save the network parameters after device reboot or
// power-off. NOTE: If this option is set to ZB_TRUE then do full device erase
// for all network devices before running other samples.
#define ERASE_PERSISTENT_CONFIG    ZB_FALSE

// LED
#define ZIGBEE_NETWORK_STATE_LED   0      // on: disconnected, blinking: identify, off: normal operation

// Button
#define BUTTON_0				BIT(0) // short press: identify, long press: factory reset

// Hall sensor
static const struct gpio_dt_spec hall_sensor = GPIO_DT_SPEC_GET(DT_NODELABEL(hall_sensor_input), gpios);

// Temperature and humidity sensor
const struct device *const hdc20 = DEVICE_DT_GET_ONE(ti_hdc2080);

// read and report battery voltage after an initial delay after joining the network 
// then read and report battery voltage after the specified period elapses.
#define BATTERY_CHECK_PERIOD_MSEC (1000 * 60 * 60 * 6) // 6 hours
#define BATTERY_CHECK_INITIAL_DELAY_MSEC (1000 * 60 * 1) // 1 minute

#define TEMP_HUMIDITY_CHECK_PERIOD_MSEC (1000 * 60 * 5) // 5 minutes
#define TEMP_HUMIDITY_CHECK_INITIAL_DELAY_MSEC (1000 * 10) // 10 seconds

#define REJOIN_ATTEMPT_PERIOD_MSEC (1000 * 60 * 5) // 5 minutes
#define REJOIN_ATTEMPT_INITIAL_DELAY_MSEC (1000 * 30) // 30 seconds

#define CONTACT_LED_INDICATION_DURATION_MSEC 500  // 500ms LED flash

// Zigbee Cluster Library 4.4.2.2.1.1: MeasuredValue = 100x temperature in degrees Celsius */
#define ZCL_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_MULTIPLIER 100
// Zigbee Cluster Library 4.7.2.1.1: MeasuredValue = 100x water content in % */
#define ZCL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_MULTIPLIER 100

//---------------------------------------------------------------------------------------------
// typedefs
//

// attribute storage for battery power config
struct zb_zcl_power_attrs {
    zb_uint8_t voltage; // Attribute 3.3.2.2.3.1
    zb_uint8_t size; // Attribute 3.3.2.2.4.2
    zb_uint8_t quantity; // Attribute 3.3.2.2.4.4
    zb_uint8_t percent_remaining; // Attribute 3.3.2.2.3.1
};

typedef struct zb_zcl_power_attrs zb_zcl_power_attrs_t;

struct zb_zcl_humidity_measurement_attrs_t
{
	zb_int16_t measure_value;
	zb_int16_t min_measure_value;
	zb_int16_t max_measure_value;
};

// attribute storage for our device
struct zb_device_ctx {
	zb_zcl_basic_attrs_ext_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
	zb_zcl_temp_measurement_attrs_t temp_attrs;
	struct zb_zcl_humidity_measurement_attrs_t humidity_attrs;
	zb_zcl_on_off_attrs_t on_off_attrs;
	zb_zcl_power_attrs_t power_attr;
};

// storage for the destination short address and endpoint number
struct dest_context {
	zb_uint8_t endpoint;
	zb_uint16_t short_addr;
};

// coin cell voltage-capacity pairs
typedef struct {
  uint16_t      voltage;
  uint8_t       capacity;
} voltage_capacity_pair_t;

// Add an attribute storage struct for On/Off server
struct zb_zcl_on_off_attrs {
    zb_bool_t on_off;
};

//---------------------------------------------------------------------------------------------
// Prototypes
//

// used to display reporting info structures if needed
zb_zcl_reporting_info_t *zb_zcl_get_reporting_info(zb_uint8_t slot_number);

void zboss_signal_handler (zb_bufid_t bufid);
static void configure_gpio (void);
static void button_handler (uint32_t button_state, uint32_t has_changed);
static void contact_send_on_off (zb_bufid_t bufid, zb_uint16_t cmd_id);
static void start_identifying (zb_bufid_t bufid);
static void identify_cb (zb_bufid_t bufid);
static void toggle_identify_led (zb_bufid_t bufid);
static void app_clusters_attr_init (void);
static void check_battery_level(zb_bufid_t bufid);
static uint8_t NiMH_CalculateLevel (uint16_t voltage);
static void configure_attribute_reporting (void);
static void check_hall_sensor_and_send_command(zb_bufid_t bufid);
static void hall_sensor_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void attempt_rejoin(zb_bufid_t bufid);
static void turn_off_led(zb_bufid_t bufid);

//---------------------------------------------------------------------------------------------
// Globals
//

LOG_MODULE_REGISTER (app, LOG_LEVEL_INF);

// Stores all cluster-related attributes
static struct zb_device_ctx dev_ctx;

// storage for the destination short address and endpoint number
static struct dest_context dest_ctx;

// Global variable to track current hall sensor state
static bool current_hall_state = false;

// Attributes setup
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_server_attr_list, 
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version, 
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id, 
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id, 
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver
);

// Declare attribute list for Identify cluster (client).
ZB_ZCL_DECLARE_IDENTIFY_CLIENT_ATTRIB_LIST(
	identify_client_attr_list
);

// Declare attribute list for Identify cluster (server).
ZB_ZCL_DECLARE_IDENTIFY_SERVER_ATTRIB_LIST(
	identify_server_attr_list,
	&dev_ctx.identify_attr.identify_time
);

// Declare attribute list for temperature measurement (server).
ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(
	temperature_measurement_attr_list,
	&dev_ctx.temp_attrs.measure_value,
	&dev_ctx.temp_attrs.min_measure_value,
	&dev_ctx.temp_attrs.max_measure_value,
	&dev_ctx.temp_attrs.tolerance
);

// Declare attribute list for humidity measurement (server).
ZB_ZCL_DECLARE_REL_HUMIDITY_MEASUREMENT_ATTRIB_LIST(
	humidity_measurement_attr_list,
	&dev_ctx.humidity_attrs.measure_value,
	&dev_ctx.humidity_attrs.min_measure_value,
	&dev_ctx.humidity_attrs.max_measure_value
);

// Declare attribute list for On/Off cluster (client).
ZB_ZCL_DECLARE_ON_OFF_CLIENT_ATTRIB_LIST(
	on_off_client_attr_list
);

// Declare attribute list for power configuration cluster (server).
// https://devzone.nordicsemi.com/f/nordic-q-a/85315/zboss-declare-power-config-attribute-list-for-battery-bat_num
#define bat_num
ZB_ZCL_DECLARE_POWER_CONFIG_BATTERY_ATTRIB_LIST_EXT(
    power_config_server_attr_list,
    &dev_ctx.power_attr.voltage,
    /*battery_size=*/&dev_ctx.power_attr.size,
    /*battery_quantity=*/&dev_ctx.power_attr.quantity,
    /*battery_rated_voltage=*/NULL,
    /*battery_alarm_mask=*/NULL,
    /*battery_voltage_min_threshold=*/NULL,
    /*battery_percentage_remaining=*/&dev_ctx.power_attr.percent_remaining,
    /*battery_voltage_threshold1=*/NULL,
    /*battery_voltage_threshold2=*/NULL,
    /*battery_voltage_threshold3=*/NULL,
    /*battery_percentage_min_threshold=*/NULL,
    /*battery_percentage_threshold1=*/NULL,
    /*battery_percentage_threshold2=*/NULL,
    /*battery_percentage_threshold3=*/NULL,
    /*battery_alarm_state=*/NULL);

// Cluster setup
ZB_DECLARE_ZICADA_CLUSTER_LIST(
	zicada_clusters, 
	basic_server_attr_list,
	identify_client_attr_list, 
	identify_server_attr_list,
	temperature_measurement_attr_list,
	humidity_measurement_attr_list,
	on_off_client_attr_list,
	power_config_server_attr_list
);

// Declare endpoint
ZB_DECLARE_ZICADA_EP(
	zicada_ep, 
	SOURCE_ENDPOINT,
	zicada_clusters
);

// Declare application's device context (list of registered endpoints) for Zicada sensor.
ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	zicada_ctx, 
	zicada_ep
);

// This allows for the initial values to be set correctly
double measured_temperature = 0;
double measured_humidity = 0;

// Voltage - Capacity pair table from thunderboard react
// Algorithm assumes the values are arranged in a descending order.
// The values in the table are base on the discharge curve from lygte-info.dk:
// https://lygte-info.dk/review/batteries2012/Ikea%20Ladda%20AA%202450mAh%20%28White%29%20UK.html
// Table modified for zigbee half percent steps.
static voltage_capacity_pair_t vcPairs[] =
{ { 1450, 100 }, { 1350, 92 }, { 1300, 78 }, { 1250, 24 }, { 1220, 13 },
  { 1160, 5 }, { 1100, 2 }, { 900, 0 } };

//---------------------------------------------------------------------------------------------
// main
//

int main (void)
{
	LOG_INF ("Starting Zicada sensor");

	// initialize
	configure_gpio ();

	// init HDC20xx
	if (!device_is_ready(hdc20)) {
		LOG_ERR("HDC20xx: device not ready");
		return 0;
	} else LOG_INF("HDC20xx: device ready");
	
	// get initial temperature and humidity
	struct sensor_value temp, humidity;
	sensor_sample_fetch(hdc20);
	sensor_channel_get(hdc20, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(hdc20, SENSOR_CHAN_HUMIDITY, &humidity);
	measured_temperature = sensor_value_to_double(&temp);
	measured_humidity = sensor_value_to_double(&humidity);
	LOG_INF("Temp = %f C, RH = %f", measured_temperature, measured_humidity);

	// init Zigbee
	register_factory_reset_button (BUTTON_0);
	zigbee_erase_persistent_storage (ERASE_PERSISTENT_CONFIG);
	zb_set_ed_timeout (ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout (ZB_MILLISECONDS_TO_BEACON_INTERVAL(3600*1000));

	// send things to endpoint 1 on the coordinator
	dest_ctx.short_addr = DEST_SHORT_ADDR;
	dest_ctx.endpoint = DEST_ENDPOINT;

	// configure for lowest power
	zigbee_configure_sleepy_behavior (true);
	power_down_unused_ram ();

	// register switch device context (endpoints)
	ZB_AF_REGISTER_DEVICE_CTX (&zicada_ctx);

	// initialize application clusters
	app_clusters_attr_init ();

	// register handlers to identify notifications
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(SOURCE_ENDPOINT, identify_cb);

	// start Zigbee default thread
	zigbee_enable ();

	LOG_INF ("Zicada sensor started");

	// suspend main thread
	while (1) {
		k_sleep (K_FOREVER);
	}
}

//---------------------------------------------------------------------------------------------
// Temperature and humidity check routine
//

static void check_temp_humidity(zb_bufid_t bufid){

	ZVUNUSED(bufid);

	sensor_sample_fetch(hdc20);

	int16_t temperature_attribute = 0;

	struct sensor_value temp;
	sensor_channel_get(hdc20, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	measured_temperature = sensor_value_to_double(&temp);
	
	// Convert measured value to attribute value, as specified in ZCL
	temperature_attribute =
		(int16_t)(measured_temperature *
			  ZCL_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_MULTIPLIER);
	//LOG_INF("Attribute T:%10d", temperature_attribute);

	// Set ZCL attribute
	zb_zcl_status_t status = zb_zcl_set_attr_val(
		SOURCE_ENDPOINT,							// 1
		ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,			// 0x0402
		ZB_ZCL_CLUSTER_SERVER_ROLE,					// 1
		ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
		(zb_uint8_t *)&temperature_attribute,
		ZB_FALSE
	);
	if (status) {
		LOG_ERR("Failed to set ZCL attribute: %d", status);
	} else{
		LOG_INF("Temperature attribute update: %.2f C", measured_temperature);
		//if(measured_temperature++ >= 50) measured_temperature = 0; // increment temperature for testing
	}

	int16_t humidity_attribute = 0;
	
	struct sensor_value humidity;
	sensor_channel_get(hdc20, SENSOR_CHAN_HUMIDITY, &humidity);
	measured_humidity = sensor_value_to_double(&humidity);
	
	// Convert measured value to attribute value, as specified in ZCL
	humidity_attribute =
		(int16_t)(measured_humidity *
			  ZCL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_MULTIPLIER);
	//LOG_INF("Attribute H:%10d", humidity_attribute);

	// Set ZCL attribute
	status = zb_zcl_set_attr_val(
		SOURCE_ENDPOINT,								// 1
		ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,		// 0x0405
		ZB_ZCL_CLUSTER_SERVER_ROLE,						// 1
		ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
		(zb_uint8_t *)&humidity_attribute,
		ZB_FALSE
	);
	if (status) {
		LOG_ERR("Failed to set ZCL attribute: %d", status);
	} else{
		LOG_INF("Humidity attribute update: %.2f%%", measured_humidity);
	}

	if(ZB_JOINED()){
		zb_ret_t zb_err = ZB_SCHEDULE_APP_ALARM(
			check_temp_humidity, 0,
			ZB_MILLISECONDS_TO_BEACON_INTERVAL(TEMP_HUMIDITY_CHECK_PERIOD_MSEC));
		if (zb_err) LOG_ERR("Failed to schedule temperature & humidity check alarm: %d", zb_err);
		else LOG_INF("Scheduled next temperature & humidity check alarm in %ds", TEMP_HUMIDITY_CHECK_PERIOD_MSEC/1000);
	}
}

//---------------------------------------------------------------------------------------------
// use the adc to periodically read the battery voltage on vdd pin and update the 
// battery voltage attribute. if joined to a network, send the attribute report.
// using the old nrfx saadc driver because it permits the adc to be shutdown between
// samples. the zephyr saadc driver does not have this capability.
//

#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6

// Battery level update routine
static void check_battery_level(zb_bufid_t bufid){

	nrfx_err_t status;
	nrfx_saadc_channel_t channel;
	uint16_t sample;

	// initialize adc
	status = nrfx_saadc_init (NRFX_SAADC_CONFIG_IRQ_PRIORITY);

	channel.channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	channel.channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	channel.channel_config.gain       = NRF_SAADC_GAIN1_6;
	channel.channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
	channel.channel_config.acq_time   = NRFX_SAADC_DEFAULT_ACQTIME;
	channel.channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
	channel.channel_config.burst      = NRF_SAADC_BURST_DISABLED;
	channel.pin_p                     = NRF_SAADC_INPUT_AIN7; // AIN7 = P0.31
	channel.pin_n                     = NRF_SAADC_INPUT_DISABLED;
	channel.channel_index             = 0;

    status = nrfx_saadc_channel_config (&channel);

	status = nrfx_saadc_simple_mode_set ((1<<0),
                                         NRF_SAADC_RESOLUTION_14BIT,
                                         NRF_SAADC_OVERSAMPLE_8X,
                                         NULL);
        
    status = nrfx_saadc_buffer_set (&sample, 1);

	// read sample
    status = nrfx_saadc_mode_trigger ();

	// shutdown adc to save power
	nrfx_saadc_uninit ();

	// convert to millivolts
	int32_t resolution = 14;
	int32_t gainrecip = 6;
	int32_t ref_mv = 600;
	int32_t adc_mv = (sample * ref_mv * gainrecip) >> resolution;

	// convert to percentage remaining
	zb_uint8_t battery_level = NiMH_CalculateLevel(adc_mv);
	//LOG_INF ("adc: %04x / %d mV / %d", sample, adc_mv, battery_level);

	zb_uint8_t percentage_attribute = battery_level * 2;
	// update percentage remaining attribute value
    zb_zcl_status_t stat = zb_zcl_set_attr_val (SOURCE_ENDPOINT,
                         ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                         ZB_ZCL_CLUSTER_SERVER_ROLE, 
                         ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                         &percentage_attribute, 
                         ZB_FALSE);
	if (stat) {
		LOG_ERR("Failed to set battery attribute: %d", stat);
	} else{
		LOG_INF("battery attribute update: %d mV / %d%%", adc_mv, battery_level);
	}

	//Schedule next alarm
	if(ZB_JOINED()){
		zb_ret_t zb_err = ZB_SCHEDULE_APP_ALARM(
			check_battery_level, 0,
			ZB_MILLISECONDS_TO_BEACON_INTERVAL(BATTERY_CHECK_PERIOD_MSEC));
		if (zb_err) LOG_ERR("Failed to schedule battery check alarm: %d", zb_err);
		else LOG_INF("Scheduled next battery check alarm in %ds", BATTERY_CHECK_PERIOD_MSEC/1000);
	}
}

//---------------------------------------------------------------------------------------------
// zigbee stack event handler
//

void zboss_signal_handler(zb_bufid_t bufid){

	static bool lastJoin = false;

	// Let default signal handler process the signal
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	// All callbacks should either reuse or free passed buffers.
	// If bufid == 0, the buffer is invalid (not passed).
	if (bufid) {
		zb_buf_free(bufid);
	}

	// once joined, set the poll and battery voltage intervals to an hour.
	// if using a sparkfun board with a spi flash chip, drop the flash chip 
	// into power down mode again just in case missed it the first time.
	bool thisJoin = ZB_JOINED();
	if ((lastJoin == false) && (thisJoin == true)) {
		LOG_INF ("joined network!");
		dk_set_led_off (ZIGBEE_NETWORK_STATE_LED);
		zb_zdo_pim_set_long_poll_interval (3600*1000);
		configure_attribute_reporting ();
		
		// Start temperature and humidity checking
		zb_ret_t err = RET_OK;
		err = ZB_SCHEDULE_APP_ALARM(check_temp_humidity, 0, ZB_MILLISECONDS_TO_BEACON_INTERVAL(TEMP_HUMIDITY_CHECK_INITIAL_DELAY_MSEC));
		if (err) LOG_ERR("Failed to schedule temperature & humidity check alarm: %d", err);
		else LOG_INF("Scheduled first temperature & humidity check alarm in %d s", TEMP_HUMIDITY_CHECK_INITIAL_DELAY_MSEC/1000);

		// Start battery level checking
		err = RET_OK;
		err = ZB_SCHEDULE_APP_ALARM(check_battery_level, 0,	ZB_MILLISECONDS_TO_BEACON_INTERVAL(BATTERY_CHECK_INITIAL_DELAY_MSEC));
		if (err) LOG_ERR("Failed to schedule battery check alarm: %d", err);
		else LOG_INF("Scheduled first battery check alarm in %d s", BATTERY_CHECK_INITIAL_DELAY_MSEC/1000);

	} else if ((lastJoin == true) && (thisJoin == false)) {
		LOG_INF ("left network!");
		// no longer joined, turn on network state led and stop reading battery voltage
		dk_set_led_on (ZIGBEE_NETWORK_STATE_LED);

		zb_ret_t err = RET_OK;
		err = ZB_SCHEDULE_APP_ALARM(attempt_rejoin, 0, ZB_MILLISECONDS_TO_BEACON_INTERVAL(REJOIN_ATTEMPT_INITIAL_DELAY_MSEC));
		if (err) LOG_ERR("Failed to schedule rejoin alarm: %d", err);
		else LOG_INF("Scheduled first rejoin alarm in %d s", REJOIN_ATTEMPT_INITIAL_DELAY_MSEC);
	}
	lastJoin = thisJoin;

}

//---------------------------------------------------------------------------------------------
// configure attribute reporting
//

#define RPT_MIN 0x0001
#define RPT_MAX 0xFFFE

static void configure_attribute_reporting (void){

	// If the maximum reporting interval is set to 0xffff then the device shall not issue any 
	// reports for the attribute. If it is set to 0x0000 and minimum reporting interval is set 
	// to something other than 0xffff then the device shall not do periodic reporting.
	// It can still send reports on value change in the last case, but not periodic.

	zb_zcl_reporting_info_t reporting_info;
	zb_ret_t status;

	memset(&reporting_info, 0, sizeof(reporting_info));
	reporting_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting_info.ep = SOURCE_ENDPOINT;
	reporting_info.cluster_id = ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
	reporting_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting_info.attr_id = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
	reporting_info.dst.short_addr = 0x0000;
	reporting_info.dst.endpoint = 1;
	reporting_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting_info.u.send_info.min_interval = RPT_MIN;
	reporting_info.u.send_info.max_interval = RPT_MAX;
	reporting_info.u.send_info.delta.u16 = 0x00;
	reporting_info.u.send_info.reported_value.u16 = 0;
	reporting_info.u.send_info.def_min_interval = RPT_MIN;
	reporting_info.u.send_info.def_max_interval = RPT_MAX;
	status = zb_zcl_put_reporting_info(&reporting_info, ZB_TRUE); 
	if (status == RET_OK) {
        LOG_INF("Temperature reporting configured successfully");
    } else {
        LOG_ERR("Failed to configure temperature reporting: %d", status);
    }

	memset(&reporting_info, 0, sizeof(reporting_info));
	reporting_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting_info.ep = SOURCE_ENDPOINT;
	reporting_info.cluster_id = ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
	reporting_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting_info.attr_id = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
	reporting_info.dst.short_addr = 0x0000;
	reporting_info.dst.endpoint = 1;
	reporting_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting_info.u.send_info.min_interval = RPT_MIN;
	reporting_info.u.send_info.max_interval = RPT_MAX;
	reporting_info.u.send_info.delta.u16 = 0x00;
	reporting_info.u.send_info.reported_value.u16 = 0;
	reporting_info.u.send_info.def_min_interval = RPT_MIN;
	reporting_info.u.send_info.def_max_interval = RPT_MAX;
	status = zb_zcl_put_reporting_info(&reporting_info, ZB_TRUE);  
	if (status == RET_OK) {
        LOG_INF("Humidity reporting configured successfully");
    } else {
        LOG_ERR("Failed to configure humidity reporting: %d", status);
    }

	memset(&reporting_info, 0, sizeof(reporting_info));
	reporting_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting_info.ep = SOURCE_ENDPOINT;
	reporting_info.cluster_id = ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
	reporting_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting_info.attr_id = ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
	reporting_info.dst.short_addr = 0x0000;
	reporting_info.dst.endpoint = 1;
	reporting_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting_info.u.send_info.min_interval = RPT_MIN;
	reporting_info.u.send_info.max_interval = RPT_MAX;
	reporting_info.u.send_info.delta.u8 = 0x00;
	reporting_info.u.send_info.reported_value.u8 = 0;
	reporting_info.u.send_info.def_min_interval = RPT_MIN;
	reporting_info.u.send_info.def_max_interval = RPT_MAX;
	status = zb_zcl_put_reporting_info(&reporting_info, ZB_TRUE); 
	if (status == RET_OK) {
        LOG_INF("Power reporting configured successfully");
    } else {
        LOG_ERR("Failed to configure power reporting: %d", status);
    }	
}

//---------------------------------------------------------------------------------------------
// configure LEDs and buttons
//

static void configure_gpio (void){

	int err;

	err = dk_buttons_init (button_handler);
	if (err) LOG_ERR ("Cannot init buttons (err: %d)", err);

	err = dk_leds_init ();
	if (err) LOG_ERR ("Cannot init LEDs (err: %d)", err);

    if (!gpio_is_ready_dt(&hall_sensor)) LOG_ERR("Hall sensor GPIO device not ready");
    
    err = gpio_pin_configure_dt(&hall_sensor, GPIO_INPUT);
    if (err != 0) LOG_ERR("Failed to configure hall sensor GPIO: %d", err);

	static struct gpio_callback hall_sensor_cb_data;
	gpio_pin_interrupt_configure_dt(&hall_sensor, GPIO_INT_LEVEL_LOW); // much lower power than EDGE_BOTH
	gpio_init_callback(&hall_sensor_cb_data, hall_sensor_interrupt_callback, BIT(hall_sensor.pin));
	gpio_add_callback(hall_sensor.port, &hall_sensor_cb_data);
	
	// turn led on until network is joined
	dk_set_led_on (ZIGBEE_NETWORK_STATE_LED); 
}


//---------------------------------------------------------------------------------------------
// set default values of attributes in the application's clusters
//

static void app_clusters_attr_init (void){

	// Basic cluster attributes data.
	dev_ctx.basic_attr.zcl_version		= ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source		= ZB_ZCL_BASIC_POWER_SOURCE_UNKNOWN;
    dev_ctx.basic_attr.power_source		= ZICADA_INIT_BASIC_POWER_SOURCE;
    dev_ctx.basic_attr.stack_version	= ZICADA_INIT_BASIC_STACK_VERSION;
    dev_ctx.basic_attr.hw_version		= ZICADA_INIT_BASIC_HW_VERSION;
	dev_ctx.basic_attr.ph_env 			= ZICADA_INIT_BASIC_PH_ENV;

    // Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
    // contain string length without trailing zero.
    //
    // For example "test" string wil be encoded as:
    //   [(0x4), 't', 'e', 's', 't']

    ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.mf_name,
                          ZICADA_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(ZICADA_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.model_id,
                          ZICADA_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(ZICADA_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.date_code,
                          ZICADA_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(ZICADA_INIT_BASIC_DATE_CODE));


    ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.location_id,
                          ZICADA_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(ZICADA_INIT_BASIC_LOCATION_DESC));

	// Identify cluster attributes data.
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;	

	// Power config attributes data.
	dev_ctx.power_attr.voltage               = ZB_ZCL_POWER_CONFIG_BATTERY_VOLTAGE_INVALID;
	dev_ctx.power_attr.size                  = ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_OTHER;
	dev_ctx.power_attr.quantity              = 1;
	dev_ctx.power_attr.percent_remaining     = ZB_ZCL_POWER_CONFIG_BATTERY_REMAINING_UNKNOWN;

	/* Temperature */
	dev_ctx.temp_attrs.measure_value = measured_temperature * ZCL_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_MULTIPLIER;
	dev_ctx.temp_attrs.min_measure_value = ZB_ZCL_TEMP_MEASUREMENT_MIN_VALUE_DEFAULT_VALUE;
	dev_ctx.temp_attrs.max_measure_value = ZB_ZCL_TEMP_MEASUREMENT_MAX_VALUE_DEFAULT_VALUE;
	dev_ctx.temp_attrs.tolerance = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

	/* Humidity */
	dev_ctx.humidity_attrs.measure_value = measured_humidity * ZCL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_MULTIPLIER;
	dev_ctx.humidity_attrs.min_measure_value = ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_DEFAULT_VALUE;
	dev_ctx.humidity_attrs.max_measure_value = ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_DEFAULT_VALUE;

	/* onOff */
	dev_ctx.on_off_attrs.on_off = ZB_ZCL_ON_OFF_IS_ON;
}

//---------------------------------------------------------------------------------------------
// button event handler
//

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	// inform default signal handler about user input at the device
	user_input_indicate();

    // check for start of factory reset
	check_factory_reset_button(button_state, has_changed);

	if (BUTTON_0 & has_changed & ~button_state) {
		// button 0 released: if not factory reset, enter identify mode
		if (!was_factory_reset_done ()) {
			// Button released before Factory Reset, Start identification mode
			ZB_SCHEDULE_APP_CALLBACK (start_identifying, 0);
		}
	}
}

//---------------------------------------------------------------------------------------------
// start identifying
//

static void start_identifying (zb_bufid_t bufid){

	ZVUNUSED(bufid);

	if (ZB_JOINED()) {
		// Check if endpoint is in identifying mode,
		// if not, put desired endpoint in identifying mode.
		if (dev_ctx.identify_attr.identify_time ==
		    ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE) {

			zb_ret_t zb_err_code = zb_bdb_finding_binding_target(SOURCE_ENDPOINT);

			if (zb_err_code == RET_OK) {
				LOG_INF("Enter identify mode");
			} else if (zb_err_code == RET_INVALID_STATE) {
				LOG_WRN("RET_INVALID_STATE - Cannot enter identify mode");
			} else {
				ZB_ERROR_CHECK(zb_err_code);
			}
		} else {
			LOG_INF("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	} else {
		LOG_WRN("Device not in a network - cannot enter identify mode");
	}
}

//---------------------------------------------------------------------------------------------
// identify callback
// 

static void identify_cb (zb_bufid_t bufid){

	zb_ret_t zb_err_code;

	if (bufid) {
		/* Schedule a self-scheduling function that will toggle the LED. */
		ZB_SCHEDULE_APP_CALLBACK(toggle_identify_led, bufid);
	} else {
		/* Cancel the toggling function alarm and turn off LED. */
		zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(toggle_identify_led, ZB_ALARM_ANY_PARAM);
		ZVUNUSED(zb_err_code);

		/* Update network status/idenitfication LED. */
		if (ZB_JOINED()) {
			dk_set_led_off(ZIGBEE_NETWORK_STATE_LED);
		} else {
			dk_set_led_on(ZIGBEE_NETWORK_STATE_LED);
		}
	}
}

//---------------------------------------------------------------------------------------------
// send contact on off command
//

static void contact_send_on_off (zb_bufid_t bufid, zb_uint16_t cmd_id){

	ZB_ZCL_ON_OFF_SEND_REQ(bufid,
		dest_ctx.short_addr,
		ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		dest_ctx.endpoint,
		SOURCE_ENDPOINT,
		ZB_AF_HA_PROFILE_ID,
		ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
		cmd_id,
		NULL);
}

//---------------------------------------------------------------------------------------------
// toggle identify led
//

static void toggle_identify_led (zb_bufid_t bufid){

	static int blink_status;

	dk_set_led(ZIGBEE_NETWORK_STATE_LED, (++blink_status) % 2);
	ZB_SCHEDULE_APP_ALARM(toggle_identify_led, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(100));
}

//---------------------------------------------------------------------------------------------
// Hall sensor interrupt callback
//

static void hall_sensor_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins){

    // Immediately disable interrupt to prevent re-triggering
    gpio_pin_interrupt_configure_dt(&hall_sensor, GPIO_INT_DISABLE);

	// Read new state
    bool new_state = gpio_pin_get_dt(&hall_sensor);
	
	// Only process if state actually changed
    if (new_state != current_hall_state) {
        current_hall_state = new_state;
        
        // Schedule the command sending
        zb_ret_t zb_err = ZB_SCHEDULE_APP_CALLBACK(check_hall_sensor_and_send_command, 0);
        if (zb_err) {
            LOG_ERR("Failed to schedule hall sensor callback: %d", zb_err);
        }
    }

	// Configure interrupt for the opposite level to avoid edge interrupt
    if (!current_hall_state) {
        // Currently high, wait for low
        gpio_pin_interrupt_configure_dt(&hall_sensor, GPIO_INT_LEVEL_LOW);
    } else {
        // Currently low, wait for high  
        gpio_pin_interrupt_configure_dt(&hall_sensor, GPIO_INT_LEVEL_HIGH);
    }
}

//---------------------------------------------------------------------------------------------
// Function to check hall sensor and send commands if needed
//

static void check_hall_sensor_and_send_command(zb_bufid_t bufid){

    static bool previous_state = false;
    bool current_state = gpio_pin_get_dt(&hall_sensor);
    
    // Only act on state changes
    if (current_state != previous_state) {
        zb_uint16_t cmd_id;
        zb_ret_t zb_err_code;

		// Always cancel any existing LED alarm first
        ZB_SCHEDULE_APP_ALARM_CANCEL(turn_off_led, ZB_ALARM_ANY_PARAM);
        
        // Turn on LED for indication
        dk_set_led_on(ZIGBEE_NETWORK_STATE_LED);

		// Schedule LED to turn off
        zb_err_code = ZB_SCHEDULE_APP_ALARM(
            turn_off_led, 0,
            ZB_MILLISECONDS_TO_BEACON_INTERVAL(CONTACT_LED_INDICATION_DURATION_MSEC)
		);
        
        if (current_state) {
            // Hall sensor activated (contact closed): send OFF command
            cmd_id = ZB_ZCL_CMD_ON_OFF_OFF_ID;
            LOG_INF("Hall sensor activated - sending OFF command");
        } else {
            // Hall sensor deactivated (contact opened): send ON command  
            cmd_id = ZB_ZCL_CMD_ON_OFF_ON_ID;
            LOG_INF("Hall sensor deactivated - sending ON command");
        }
        
        // Send the command
        zb_err_code = zb_buf_get_out_delayed_ext(contact_send_on_off, cmd_id, 0);
        ZB_ERROR_CHECK(zb_err_code);
        
        previous_state = current_state;
    }
}

//---------------------------------------------------------------------------------------------
// Calculate battery level based on the cell voltage (from thunderboard react)
//

uint8_t NiMH_CalculateLevel (uint16_t voltage){

  uint32_t res = 0;
  uint8_t i;

  // Iterate through voltage/capacity table until correct interval is found.
  // Then interpolate capacity within that interval based on a linear approximation
  // between the capacity at the low and high end of the interval.
  for (i = 0; i < (sizeof(vcPairs) / sizeof(voltage_capacity_pair_t)); i++) {
    if (voltage > vcPairs[i].voltage) {
      if (i == 0) {
        // Higher than maximum voltage in table.
        return vcPairs[0].capacity;
      } else {
        // Calculate the capacity by interpolation.
        res = (voltage - vcPairs[i].voltage)
              * (vcPairs[i - 1].capacity - vcPairs[i].capacity)
              / (vcPairs[i - 1].voltage - vcPairs[i].voltage);
        res += vcPairs[i].capacity;
        return (uint8_t)res;
      }
    }
  }
  // Below the minimum voltage in the table.
  return vcPairs[sizeof(vcPairs) / sizeof(voltage_capacity_pair_t) - 1].capacity;
}

//---------------------------------------------------------------------------------------------
// Rejoin attempt routine
//

static void attempt_rejoin(zb_bufid_t bufid){

	ZVUNUSED(bufid);

	if(ZB_JOINED()){
		LOG_INF("Already joined - no need for rejoin.");
	} 
	else{
		LOG_INF("Waking up Zigbee Stack for rejoin.");
		user_input_indicate();

		zb_ret_t zb_err = ZB_SCHEDULE_APP_ALARM(
		attempt_rejoin, 0,
		ZB_MILLISECONDS_TO_BEACON_INTERVAL(REJOIN_ATTEMPT_PERIOD_MSEC));
		if (zb_err) {
			LOG_ERR("Failed to schedule rejoin alarm: %d", zb_err);
		}
		else LOG_INF("Scheduled next rejoin alarm in %ds", REJOIN_ATTEMPT_PERIOD_MSEC/1000);
	}	
}

//---------------------------------------------------------------------------------------------
// Turn off the LED via a timer
//

static void turn_off_led(zb_bufid_t bufid){

    ZVUNUSED(bufid);
    // Restore network status LED state
    if (ZB_JOINED()) dk_set_led_off(ZIGBEE_NETWORK_STATE_LED);
    else dk_set_led_on(ZIGBEE_NETWORK_STATE_LED);
}