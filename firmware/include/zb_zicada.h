#ifndef __ZB_ZICADA_H__
#define __ZB_ZICADA_H__

// Temperature sensor device ID
#define ZB_TEMPERATURE_SENSOR_DEVICE_ID 0x0302  

// Device version
#define ZB_DEVICE_VER_TEMPERATURE_SENSOR 0

// Zicada sensor numer of IN (server) clusters
#define ZB_ZICADA_IN_CLUSTER_NUM 5

// Zicada sensor number of OUT (client) clusters
#define ZB_ZICADA_OUT_CLUSTER_NUM 2

// Zicada sensor total number of (IN+OUT) clusters
#define ZB_ZICADA_CLUSTER_NUM (ZB_ZICADA_IN_CLUSTER_NUM + ZB_ZICADA_OUT_CLUSTER_NUM)

// Number of attributes for reporting on Zicada sensor
// battery percentage remaining, battery alarm + battery voltage
#define ZB_ZICADA_REPORT_ATTR_COUNT (ZB_ZCL_POWER_CONFIG_REPORT_ATTR_COUNT + 1)

// Declare cluster list for Zicada sensor
//
// cluster_list_name - cluster list variable name
// basic_server_attr_list - attribute list for Basic cluster (server role)
// identify_server_attr_list - attribute list for Identify cluster (server role)
// identify_client_attr_list - attribute list for Identify cluster (client role)
// temperature_measurement_attr_list - attribute list for temperature cluster (server role)
// humidity_measurement_attr_list - attribute list for humidity cluster (server role)
// on_off_client_attr_list - attribute list for On/Off cluster (client role)
// power_config_server_attr_list - attribute list for Power COnfig cluster (server role)

#define ZB_DECLARE_ZICADA_CLUSTER_LIST(			  									\
		cluster_list_name,						      								\
		basic_server_attr_list,														\
		identify_client_attr_list,													\
		identify_server_attr_list,													\
		temperature_measurement_attr_list,											\
		humidity_measurement_attr_list,												\
		on_off_client_attr_list,													\
		power_config_server_attr_list)												\
zb_zcl_cluster_desc_t cluster_list_name[] =											\
{										  											\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_BASIC,													\
		ZB_ZCL_ARRAY_SIZE(basic_server_attr_list, zb_zcl_attr_t),					\
		(basic_server_attr_list),													\
		ZB_ZCL_CLUSTER_SERVER_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_IDENTIFY,													\
		ZB_ZCL_ARRAY_SIZE(identify_client_attr_list, zb_zcl_attr_t),				\
		(identify_client_attr_list),												\
		ZB_ZCL_CLUSTER_CLIENT_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_IDENTIFY,													\
		ZB_ZCL_ARRAY_SIZE(identify_server_attr_list, zb_zcl_attr_t),				\
		(identify_server_attr_list),												\
		ZB_ZCL_CLUSTER_SERVER_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,											\
		ZB_ZCL_ARRAY_SIZE(temperature_measurement_attr_list, zb_zcl_attr_t),		\
		(temperature_measurement_attr_list),										\
		ZB_ZCL_CLUSTER_SERVER_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,									\
		ZB_ZCL_ARRAY_SIZE(humidity_measurement_attr_list, zb_zcl_attr_t),			\
		(humidity_measurement_attr_list),											\
		ZB_ZCL_CLUSTER_SERVER_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_ON_OFF,													\
		ZB_ZCL_ARRAY_SIZE(on_off_client_attr_list, zb_zcl_attr_t),					\
		(on_off_client_attr_list),													\
		ZB_ZCL_CLUSTER_CLIENT_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	),																				\
	ZB_ZCL_CLUSTER_DESC(															\
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG,												\
		ZB_ZCL_ARRAY_SIZE(power_config_server_attr_list, zb_zcl_attr_t),			\
		(power_config_server_attr_list),											\
		ZB_ZCL_CLUSTER_SERVER_ROLE,													\
		ZB_ZCL_MANUF_CODE_INVALID													\
	)																				\
}

// Declare simple descriptor for Zicada sensor
//
// ep_name - endpoint variable name
// ep_id - endpoint ID
// in_clust_num - number of supported input clusters
// out_clust_num - number of supported output clusters

#define ZB_ZCL_DECLARE_ZICADA_SIMPLE_DESC(											\
	ep_name, ep_id, in_clust_num, out_clust_num)									\
	ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);							\
	ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num) simple_desc_##ep_name =		\
	{																				\
		ep_id,																		\
		ZB_AF_HA_PROFILE_ID,														\
		ZB_TEMPERATURE_SENSOR_DEVICE_ID,											\
		ZB_DEVICE_VER_TEMPERATURE_SENSOR,											\
		0,																			\
		in_clust_num,																\
		out_clust_num,																\
		{																			\
			ZB_ZCL_CLUSTER_ID_BASIC,												\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,												\
			ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,										\
			ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,								\
			ZB_ZCL_CLUSTER_ID_POWER_CONFIG,											\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,												\
			ZB_ZCL_CLUSTER_ID_ON_OFF												\
		}																			\
	}

// Declare endpoint for Zicada sensor
//
// ep_name - endpoint variable name
// ep_id - endpoint ID
// cluster_list - endpoint cluster list

#define ZB_DECLARE_ZICADA_EP(ep_name, ep_id, cluster_list)						\
	ZB_ZCL_DECLARE_ZICADA_SIMPLE_DESC(ep_name, ep_id,							\
		  ZB_ZICADA_IN_CLUSTER_NUM, ZB_ZICADA_OUT_CLUSTER_NUM);					\
	ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info## ep_name,				\
		ZB_ZICADA_REPORT_ATTR_COUNT);											\
	ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID, 0, NULL,	\
		ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list,	\
		(zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,						\
		ZB_ZICADA_REPORT_ATTR_COUNT, reporting_info## ep_name,					\
		0, NULL) // No CVC ctx

#endif // __ZB_ZICADA_H__
