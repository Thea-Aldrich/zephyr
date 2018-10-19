/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <sensor.h>
#include <device.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>


static u8_t temperature = 0;		//This variable holds the current measured temperature

/*
 * SUMMARY: Uses helper function to read the current measured temperature into a response buffer when requested by
 * 	the other side of the bluetooth connection.
 */
static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						 void *buf, u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn,                 // Handed to us by the bluetooth stack
			        attr,                 // Handed to us by the bluetooth stack
			        buf,                  // Handed to us by the bluetooth stack
			        len,                  // Handed to us by the bluetooth stack
			        offset,               // Handed to us by the bluetooth stack
			        &temperature,         // Value to be pushed into buffer to send to requestor (other side of connection)
				sizeof(temperature)   // Size of value to be pushed
				        );
}


/* SUMMARY: This struct defines a Generic Attribute Profile (GATT).
 * 	A GATT Profile *HAS* a Service.
 * 	A Service *HAS* a characteristic.
 * 	A Characteristic is a piece of data.
 * SIMPLE Temp Service Declaration
*/
static struct bt_gatt_attr attrs[] = {
		// Environmental Sensing Service bitmask defined in zephyr/include/bluetooth/uuid.h
		BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
		BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,     						//UUID defining what kind of characteristic (data) this is
	    						BT_GATT_CHRC_READ,						        // Available Actions (read,write,notify,etc)
	    						BT_GATT_PERM_READ,                              // Available Permissions
								read_temp,                                      // Read callback function
								NULL,                                           // Write callback function
								&temperature                                    // Value this characteristic represents
				               )
};

static struct bt_gatt_service measure_temperature_service = BT_GATT_SERVICE(attrs);


/*
 * SUMMARY: Reads the current temperature and notifies all clients of a new value if they have registered with
 * 	notify option.
 */
void temperature_measurement_notify(void)
{
	static u8_t temp_data[2]; 	// [0] = Type
								// [1] = data primitive (the actual temperature)

	// Poll the temperature sensor
    //sensor_channel_get(sensor_devices_array[0], temperature_sensor_config[0].chan, &temperature_data);

	// Pack the data
	temp_data[0] = 0x06; /* uint8, sensor contact */
	temp_data[1] = temperature;

	// Notify all clients/peers of a new value
	bt_gatt_notify(NULL, &attrs[1], &temp_data, sizeof(temp_data));
}



struct bt_conn *default_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		default_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	// Register temperature service
	bt_gatt_service_register(&measure_temperature_service);

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

/*
 * TEMPERATURE SENSOR CONFIGURATION SECTION
 */
#ifdef CONFIG_APDS9660_TRIGGER
K_SEM_DEFINE(sem, 0, 1);
static void trigger_handler(struct device *dev, struct sensor_trigger *trigger) {
    ARG_UNUSED(dev);
    ARG_UNUSED(trigger);

    k_sem_give(&sem);
}
#endif

struct channel_info {
	int chan;
	char *dev_name;
};

#define SENSOR_CHAN_TEMP 14 //Which pin sensor is connected to (found via trial and error)

static struct channel_info info[] = {
		{SENSOR_CHAN_TEMP, "HDC1008"},
		{SENSOR_CHAN_HUMIDITY, "HDC1008"},
		{SENSOR_CHAN_PROX, CONFIG_APDS9960_DRV_NAME},
		{SENSOR_CHAN_LIGHT, CONFIG_APDS9960_DRV_NAME}
};


void main(void)
{
	/*
	 * INTIALIZE THE TEMPERATURE SENSOR
	 */
	printk("1\n");
    // Sensor array, one element per sensor being used.
    struct device *dev[ARRAY_SIZE(info)];
    printk("2\n");
    // Struct to hold measurement from sensor
    struct sensor_value temperature_data;
    printk("3\n");

	// Bind sensor to device stuct
    for (int i = 0; i < ARRAY_SIZE(info); i++) {
    	dev[i] = device_get_binding(info[i].dev_name);
    	if (dev[i] == NULL) {
    		printk("ERROR: Sensor %s failed to initialize\n", info[i].dev_name);
    	} else {
    		printk("Sensor %s initialized\n", info[i].dev_name);
    	}
    }
    printk("4\n");

	printk("5\n");
	// Populate the first sensor value
	//sensor_sample_fetch(dev[0]);
	printk("6\n");
	//sensor_channel_get(dev[0], info[0].chan, &temperature_data);
	printk("7\n");
	//printk("SANITY CHECK: First temperature sensor reading: %d\n", temperature_data.val1);

	/*
	 * INITIALIZE THE BLUETOOTH SENSOR
	 */
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(1000);
		printk("test");
		/* Temperature measurement */
		//temperature_measurement_notify();
	}
}
