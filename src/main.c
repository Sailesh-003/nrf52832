#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <zephyr/types.h>

#define I2C_NODE DT_NODELABEL(i2c0)
#define ADPD144RI_I2C_ADDR 0x64
#define NUM_CHANNELS 3
#define MSGQ_SIZE 10

#define UNIX_TIME_START 1718000000UL   // <<-- Set this to your time!
#define LFCLK_FREQ 32768UL
#define RTC2_PRESCALER 0
#define RTC_TICKS_PER_SEC (LFCLK_FREQ / (RTC2_PRESCALER + 1))

typedef struct {
    uint32_t sample_id;
    uint64_t timestamp_ms;
    uint32_t channels[NUM_CHANNELS];
} SampleEntry;

K_MSGQ_DEFINE(sensor_msgq, sizeof(SampleEntry), MSGQ_SIZE, 4);

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);

static struct bt_uuid_128 char_uuid = BT_UUID_INIT_128(
    0x01, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);

static struct bt_conn *current_conn = NULL;
static volatile bool notify_enabled = false;
static char sensor_notify_buf[128];
static struct k_mutex notify_buf_mutex;

static void rtc2_init(void) {
    printk("[RTC2] Initializing RTC2...\n");
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED) {}
    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->PRESCALER = RTC2_PRESCALER;
    NRF_RTC2->TASKS_CLEAR = 1;
    NRF_RTC2->TASKS_START = 1;
    printk("[RTC2] RTC2 running (prescaler=%d, unix start=%lu).\n", RTC2_PRESCALER, (unsigned long)UNIX_TIME_START);
}

static uint64_t get_timestamp_ms(void) {
    uint32_t ticks = NRF_RTC2->COUNTER;
    uint64_t ms = ((uint64_t)ticks * 1000UL) / RTC_TICKS_PER_SEC;
    return ((uint64_t)UNIX_TIME_START * 1000UL) + ms;
}

static void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size) {
    time_t sec = timestamp_ms / 1000;
    uint16_t ms = timestamp_ms % 1000;
    struct tm t;
    gmtime_r(&sec, &t);
    snprintf(buffer, size, "%02d/%02d/%04d %02d:%02d:%02d.%03d",
        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
        t.tm_hour, t.tm_min, t.tm_sec, ms);
}

static void print_ble_address(void)
{
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    char buf[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, buf, sizeof(buf));
    printk("[BLE] Device BT Address: %s\n", buf);
}

static ssize_t read_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                void *buf, uint16_t len, uint16_t offset)
{
    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
    ssize_t ret = bt_gatt_attr_read(
        conn, attr, buf, len, offset,
        sensor_notify_buf, strnlen(sensor_notify_buf, sizeof(sensor_notify_buf))
    );
    k_mutex_unlock(&notify_buf_mutex);
    printk("[BLE] GATT: read_sensor (len=%u, offset=%u)\n", len, offset);
    return ret;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("[BLE] CCCD config changed: notifications %s (value=0x%x)\n",
           notify_enabled ? "ENABLED" : "DISABLED", value);
}

BT_GATT_SERVICE_DEFINE(sensor_svc,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_sensor, NULL, sensor_notify_buf),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static int adpd144ri_write_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = { reg, value >> 8, value & 0xFF };
    int ret = i2c_write(i2c_dev, buf, sizeof(buf), ADPD144RI_I2C_ADDR);
    if (ret)
        printk("[I2C] Write reg 0x%02X failed: %d\n", reg, ret);
    return ret;
}
static int adpd144ri_read_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    int ret = i2c_write_read(i2c_dev, ADPD144RI_I2C_ADDR, &reg, 1, data, 2);
    if (ret)
        printk("[I2C] Read reg 0x%02X failed: %d\n", reg, ret);
    else
        *value = (data[0] << 8) | data[1];
    return ret;
}
static int adpd144ri_configure_spo2_hr(const struct device *i2c_dev) {
    printk("[I2C] Configuring ADPD144RI sensor ...\n");
    int rc = 0;
    rc += adpd144ri_write_reg16(i2c_dev, 0x10, 0x0001);
    rc += adpd144ri_write_reg16(i2c_dev, 0x02, 0x0005);
    rc += adpd144ri_write_reg16(i2c_dev, 0x01, 0x009F);
    rc += adpd144ri_write_reg16(i2c_dev, 0x11, 0x30A9);
    rc += adpd144ri_write_reg16(i2c_dev, 0x12, 0x0384);
    rc += adpd144ri_write_reg16(i2c_dev, 0x15, 0x0330);
    rc += adpd144ri_write_reg16(i2c_dev, 0x14, 0x0116);
    rc += adpd144ri_write_reg16(i2c_dev, 0x18, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x19, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1A, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1B, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1E, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1F, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x20, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x21, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x23, 0x3005);
    rc += adpd144ri_write_reg16(i2c_dev, 0x24, 0x3007);
    rc += adpd144ri_write_reg16(i2c_dev, 0x25, 0x0207);
    rc += adpd144ri_write_reg16(i2c_dev, 0x30, 0x0319);
    rc += adpd144ri_write_reg16(i2c_dev, 0x31, 0x0813);
    rc += adpd144ri_write_reg16(i2c_dev, 0x35, 0x0319);
    rc += adpd144ri_write_reg16(i2c_dev, 0x36, 0x0813);
    rc += adpd144ri_write_reg16(i2c_dev, 0x39, 0x21F3);
    rc += adpd144ri_write_reg16(i2c_dev, 0x3B, 0x21F3);
    rc += adpd144ri_write_reg16(i2c_dev, 0x42, 0x1C36);
    rc += adpd144ri_write_reg16(i2c_dev, 0x44, 0x1C36);
    rc += adpd144ri_write_reg16(i2c_dev, 0x4E, 0x0040);
    rc += adpd144ri_write_reg16(i2c_dev, 0x4B, 0x0080);
    rc += adpd144ri_write_reg16(i2c_dev, 0x10, 0x0002);
    k_msleep(50);
    if (rc)
        printk("[I2C] One or more sensor config writes failed: rc=%d\n", rc);
    else
        printk("[I2C] ADPD144RI configuration done.\n");
    return rc;
}
static int adpd144ri_read_3ch(const struct device *i2c_dev, uint32_t *data) {
    int rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0006);
    if (rc) { printk("[I2C] Measurement reg set failed: %d\n", rc); return rc; }

    uint16_t ch3_l=0, ch3_h=0, ch4_l=0, ch4_h=0, bch3_l=0, bch3_h=0;

    rc |= adpd144ri_read_reg16(i2c_dev, 0x72, &ch3_l);
    rc |= adpd144ri_read_reg16(i2c_dev, 0x76, &ch3_h);
    rc |= adpd144ri_read_reg16(i2c_dev, 0x73, &ch4_l);
    rc |= adpd144ri_read_reg16(i2c_dev, 0x77, &ch4_h);
    rc |= adpd144ri_read_reg16(i2c_dev, 0x7A, &bch3_l);
    rc |= adpd144ri_read_reg16(i2c_dev, 0x7E, &bch3_h);

    if (rc) { printk("[I2C] One or more channel reads failed: rc=%d\n", rc); return rc; }

    data[0] = ((uint32_t)ch3_h << 16) | ch3_l;
    data[1] = ((uint32_t)ch4_h << 16) | ch4_l;
    data[2] = ((uint32_t)bch3_h << 16) | bch3_l;

    rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0000);
    if (rc) { printk("[I2C] Measurement reg reset failed: %d\n", rc); return rc; }
    return 0;
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "ADPD_BLE", 8),
};

static void bt_ready(int err) {
    printk("[MAIN] bt_ready callback, err=%d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("[BLE] Bluetooth initialized\n");
    print_ble_address();
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("[BLE] Advertising failed to start (err %d)\n", err);
    } else {
        printk("[BLE] Advertising started successfully!\n");
    }
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("[BLE] Connection failed (err %u)\n", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    printk("[BLE] Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("[BLE] Disconnected (reason 0x%02x)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

void sensor_thread(void) {
    static uint32_t sample_id = 1;
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    printk("[THREAD] sensor_thread started\n");

    if (!device_is_ready(i2c_dev)) {
        printk("[I2C] device %s not ready!\n", i2c_dev->name);
        printk("[SENSOR] Thread will now sleep forever (does not block system).\n");
        while (1) k_msleep(1000);
    }

    rtc2_init();

    while (1) {
        while (!notify_enabled) {
            k_msleep(200);
        }

        while (adpd144ri_configure_spo2_hr(i2c_dev) != 0) {
            printk("[SENSOR] Sensor config failed, sleeping and retrying...\n");
            k_msleep(200);
            if (!notify_enabled) break;
        }
        if (!notify_enabled) continue;

        printk("[SENSOR] Measurement loop started\n");

        while (notify_enabled) {
            SampleEntry entry;
            entry.sample_id = sample_id++;
            entry.timestamp_ms = get_timestamp_ms();
            int rc = adpd144ri_read_3ch(i2c_dev, entry.channels);
            if (rc) {
                printk("[SENSOR] I2C sensor read failed (rc=%d). Yielding...\n", rc);
                k_msleep(100);
                continue;
            }

            int ret = k_msgq_put(&sensor_msgq, &entry, K_NO_WAIT);
            if (ret) printk("[SENSOR] k_msgq_put failed: %d\n", ret);

            k_msleep(1000); // 1Hz
        }
        printk("[SENSOR] Measurement loop paused (notifications off)\n");
    }
}

void printer_thread(void) {
    printk("[THREAD] printer_thread started\n");
    SampleEntry entry;
    char ts[64];
    int dropped_count = 0;

    while (1) {
        int kret = k_msgq_get(&sensor_msgq, &entry, K_FOREVER);
        if (kret != 0) {
            printk("[PRINTER] k_msgq_get failed: %d\n", kret);
            continue;
        }
        format_timestamp(entry.timestamp_ms, ts, sizeof(ts));
        k_mutex_lock(&notify_buf_mutex, K_FOREVER);
        snprintf(sensor_notify_buf, sizeof(sensor_notify_buf), "%lu %s %lu %lu %lu",
            (unsigned long)entry.sample_id, ts,
            (unsigned long)entry.channels[0], (unsigned long)entry.channels[1], (unsigned long)entry.channels[2]);
        sensor_notify_buf[sizeof(sensor_notify_buf)-1] = '\0';
        k_mutex_unlock(&notify_buf_mutex);

        printk("[PRINTER] BLE Sample: %s\n", sensor_notify_buf);
        if (notify_enabled && current_conn) {
            int attempts = 0;
            int err;
            do {
                err = bt_gatt_notify(current_conn, &sensor_svc.attrs[1],
                                     sensor_notify_buf,
                                     strnlen(sensor_notify_buf, sizeof(sensor_notify_buf)));
                if (err == -ENOMEM) {
                    attempts++;
                    dropped_count++;
                    if (dropped_count <= 5 || dropped_count % 100 == 0) {
                        printk("[BLE] Notify error: buffer full - notification dropped (count=%d)\n", dropped_count);
                    }
                    k_msleep(100); // Wait a bit before retrying
                } else if (err) {
                    printk("[BLE] Notify error: %d\n", err);
                    break;
                } else {
                    dropped_count = 0;
                    break;
                }
            } while (err == -ENOMEM && attempts < 5);
        }
    }
}

K_THREAD_DEFINE(sensor_tid, 2048, sensor_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(printer_tid, 2048, printer_thread, NULL, NULL, NULL, 5, 0, 0);

void main(void) {
    printk("\n********* BOOTING ADPD144RI BLE SENSOR DEMO *********\n");
    k_mutex_init(&notify_buf_mutex);
    memset(sensor_notify_buf, 0, sizeof(sensor_notify_buf)); // Make sure buffer initialized
    int err = bt_enable(bt_ready);
    printk("[MAIN] Called bt_enable(), returned %d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
    }
}
