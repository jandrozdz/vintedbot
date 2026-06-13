/*
 * TrikiDumper — streams internal flash (192 KB) over BLE NUS.
 * Flash is memory-mapped on nRF52810; we read it directly as a pointer.
 * Trigger: connect and send any byte → START marker → raw flash → DONE marker.
 *
 * Target: nRF52810  (no FPU, 192 KB flash, S112 SoftDevice)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <string.h>

#define FLASH_BASE   0x00000000UL
#define FLASH_SIZE   (192U * 1024U)   /* 192 KB */
#define CHUNK        200U             /* well under typical MTU after negotiation */

static struct bt_conn *g_conn;
static volatile bool   g_dump;

/* ── BLE NUS callbacks ─────────────────────────────────────────────────────── */

static void nus_rx(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    g_dump = true;
}

static struct bt_nus_cb nus_cb = { .received = nus_rx };

/* ── connection callbacks ─────────────────────────────────────────────────── */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        g_conn = bt_conn_ref(conn);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);
    bt_conn_unref(g_conn);
    g_conn  = NULL;
    g_dump  = false;
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/* ── advertising ──────────────────────────────────────────────────────────── */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "TrikiDumper", 11),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ── flash dump ───────────────────────────────────────────────────────────── */

static void do_dump(void)
{
    const uint8_t *p = (const uint8_t *)FLASH_BASE;
    uint32_t sent = 0;
    int rc;

    bt_nus_send(g_conn, (const uint8_t *)"START\n", 6);
    k_sleep(K_MSEC(100));

    while (sent < FLASH_SIZE) {
        uint32_t n = MIN(CHUNK, FLASH_SIZE - sent);

        /* Spin-wait on flow-control pressure */
        do {
            rc = bt_nus_send(g_conn, p + sent, n);
            if (rc != 0) k_sleep(K_MSEC(10));
        } while (rc != 0 && g_conn != NULL);

        if (g_conn == NULL) return;   /* disconnected mid-dump */

        sent += n;
        k_sleep(K_MSEC(5));
    }

    bt_nus_send(g_conn, (const uint8_t *)"DONE\n", 5);
}

/* ── main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    bt_nus_init(&nus_cb);
    bt_enable(NULL);
    bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    while (1) {
        if (g_dump && g_conn) {
            g_dump = false;
            do_dump();
        }
        k_sleep(K_MSEC(50));
    }
    return 0;
}
