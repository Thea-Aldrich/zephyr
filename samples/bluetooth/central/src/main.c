/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <misc/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <misc/byteorder.h>
#include <buf.h>

static struct bt_conn *default_conn;

static void connected(struct bt_conn *conn, u8_t err)
{
    printk("Connection callback!\n");
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	/*
	 * START: Logic to grab values and pass them to RasPi
	 */

	// Poll sensors

	// Read sensors

	// Prepare for send
	// Create net buffer
	//https://docs.zephyrproject.org/latest/api/networking.html#_CPPv37net_buf

	// Copy sensor values into buffer
    
    // Send packet
	// USE RFCOMM SEND
    // https://docs.zephyrproject.org/latest/api/bluetooth.html#_CPPv318bt_rfcomm_dlc_sendP13bt_rfcomm_dlcP7net_buf
    // int bt_rfcomm_dlc_send(struct bt_rfcomm_dlc *dlc, struct net_buf *buf)


	// Disconnect
	//bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason %u)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
}

static struct bt_conn_cb conn_callbacks = {
		.connected = connected,
		.disconnected = disconnected,
};

void main(void)
{
	/*
	 * INITIALIZE BLUETOOTH RADIO
	 */
	int err; // Used to store error code
    // Turn on bluetooth radio
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

//	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
//	if (err) {
//		printk("Scanning failed to start (err %d)\n", err);
//		return;
//	}
	const bt_addr_le_t raspi = { 0, { {52, 226, 113, 235, 39, 184} } };
    default_conn = bt_conn_create_le(&raspi, BT_LE_CONN_PARAM_DEFAULT);

    err = bt_le_set_auto_conn(&raspi, BT_LE_CONN_PARAM_DEFAULT);
    if (err) {
        printk("ERROR: BLE Auto connect failed with:\n\t%d", err);
    }

    // Make Raspi address into string to print it out
    char raspi_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&raspi, raspi_addr, sizeof(raspi_addr));
	printk("BLE auto connection started: Trying to connect to %s\n", raspi_addr);
}
