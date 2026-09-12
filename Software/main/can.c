#include "can.h"
#include "esp_log.h"
#include "esp_mac.h"
/* ── J1939 constants ─────────────────────────────────────────────────────── */
#define J1939_PGN_ADDR_CLAIMED      0xEE00UL
#define J1939_ADDR_GLOBAL           0xFF     /* broadcast destination        */
#define J1939_ADDR_NULL             0xFE     /* "cannot claim" source addr   */
#define J1939_ADDR_DEFAULT          0x60     /* preferred address to try first */
#define J1939_CLAIM_TIMEOUT_MS      250      /* listen window per attempt    */

/*
 * J1939 NAME – 64-bit device identifier (SAE J1939-81).
 * Edit these fields to describe your device.
 * Identity number is derived from the chip MAC at runtime for uniqueness.
 *
 * Bit layout (LSB = bit 0):
 *  [20: 0]  Identity Number     (21 bits) — set from MAC below
 *  [31:21]  Manufacturer Code   (11 bits) — 0x7FF = unassigned/test
 *  [34:32]  ECU Instance        ( 3 bits)
 *  [39:35]  Function Instance   ( 5 bits)
 *  [47:40]  Function            ( 8 bits) — 0x7F = not defined
 *  [48]     Reserved            ( 1 bit)  — must be 0
 *  [55:49]  Vehicle System      ( 7 bits)
 *  [59:56]  Vehicle System Inst ( 4 bits)
 *  [62:60]  Industry Group      ( 3 bits) — 0 = global
 *  [63]     Arbitrary Addr Cap  ( 1 bit)  — 1 = can self-configure address
 */
#define J1939_NAME_MFR_CODE         0x7FF
#define J1939_NAME_ECU_INST         0
#define J1939_NAME_FUNC_INST        0
#define J1939_NAME_FUNCTION         0x7F
#define J1939_NAME_VEHICLE_SYS      0
#define J1939_NAME_VEH_SYS_INST     0
#define J1939_NAME_INDUSTRY_GRP     0
#define J1939_NAME_ARBI_ADDR        1        /* must be 1 to self-configure */

/* ── TX demo ─────────────────────────────────────────────────────────────── */
#define TX_INTERVAL_MS  250

static const char *TAG = "CAN";

/* Our claimed source address, set by j1939_address_claim(). */
static volatile uint8_t g_sa = J1939_ADDR_NULL;

static uint8_t fail_count = 6;  /* count of consecutive TX failures, default failed until recieve can*/
static int16_t CANPositionCommand = CAN_POSITION_INVALID; /* variable to hold the current command for the motor */
static int16_t CANPositionActual = CAN_POSITION_INVALID; /* variable to hold the current position of the motor */

/* ── J1939 helpers ───────────────────────────────────────────────────────── */

/**
 * Build the 64-bit J1939 NAME from the compile-time fields and a runtime
 * identity number (lower 21 bits of the chip MAC).
 */
static uint64_t j1939_build_name(uint32_t identity)
{
    uint64_t n = 0;
    n |= ((uint64_t)(identity              & 0x1FFFFF));
    n |= ((uint64_t)(J1939_NAME_MFR_CODE   & 0x7FF))   << 21;
    n |= ((uint64_t)(J1939_NAME_ECU_INST   & 0x007))   << 32;
    n |= ((uint64_t)(J1939_NAME_FUNC_INST  & 0x01F))   << 35;
    n |= ((uint64_t)(J1939_NAME_FUNCTION   & 0x0FF))   << 40;
    /* bit 48 = reserved, leave 0 */
    n |= ((uint64_t)(J1939_NAME_VEHICLE_SYS  & 0x07F)) << 49;
    n |= ((uint64_t)(J1939_NAME_VEH_SYS_INST & 0x00F)) << 56;
    n |= ((uint64_t)(J1939_NAME_INDUSTRY_GRP & 0x007)) << 60;
    n |= ((uint64_t)(J1939_NAME_ARBI_ADDR    & 0x001)) << 63;
    return n;
}

/**
 * Build the 29-bit J1939 CAN ID for an Address Claimed frame:
 *   Priority=6, R=0, DP=0, PF=0xEE, PS=0xFF (global), SA=sa
 */
static inline uint32_t addr_claimed_id(uint8_t sa)
{
    return (6UL << 26) | (0xEEUL << 16) | (0xFFUL << 8) | sa;
}

/**
 * Return true if the frame is an Address Claimed (PGN 0xEE00, global dest)
 * sent by source address sa.
 */
static inline bool is_addr_claimed(const twai_message_t *m, uint8_t sa)
{
    return m->extd
        && ((m->identifier >> 16) & 0xFF) == 0xEE
        && ((m->identifier >>  8) & 0xFF) == 0xFF
        &&  (m->identifier        & 0xFF) == sa;
}

/** Transmit an Address Claimed frame. */
static esp_err_t send_addr_claimed(uint8_t sa, uint64_t name)
{
    twai_message_t msg = {
        .identifier       = addr_claimed_id(sa),
        .extd             = 1,
        .rtr              = 0,
        .ss               = 0,
        .self             = 0,
        .dlc_non_comp     = 0,
        .data_length_code = 8,
    };
    for (int i = 0; i < 8; i++) {
        msg.data[i] = (name >> (i * 8)) & 0xFF;   /* little-endian NAME */
    }
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

/* ── Address claim procedure ─────────────────────────────────────────────── */

static void j1939_address_claim(uint64_t name)
{
    uint8_t sa         = J1939_ADDR_DEFAULT;
    uint8_t start_sa   = sa;
    bool    wrapped    = false;

    while (1) {
        ESP_LOGI(TAG, "Claiming SA=0x%02X …", sa);

        if (send_addr_claimed(sa, name) != ESP_OK) {
            ESP_LOGE(TAG, "TX failed – retrying");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* ── Listen window ─────────────────────────────────────────────── */
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(J1939_CLAIM_TIMEOUT_MS);
        bool yielded = false;

        while (xTaskGetTickCount() < deadline) {
            TickType_t remaining = deadline - xTaskGetTickCount();
            twai_message_t rx;

            if (twai_receive(&rx, remaining) != ESP_OK) {
                break; /* timeout – no conflict */
            }

            if (!is_addr_claimed(&rx, sa)) {
                continue; /* unrelated frame */
            }

            /* Another node is claiming the same address */
            uint64_t their_name = 0;
            for (int i = 0; i < 8; i++) {
                their_name |= (uint64_t)rx.data[i] << (i * 8);
            }

            if (their_name < name) {
                /* Lower NAME wins – we yield */
                ESP_LOGW(TAG, "SA=0x%02X lost to NAME=0x%016llX, incrementing",
                         sa, (unsigned long long)their_name);
                yielded = true;
                break;
            } else {
                /* We win – re-assert and restart the window */
                ESP_LOGI(TAG, "SA=0x%02X contested, our NAME wins, re-asserting", sa);
                send_addr_claimed(sa, name);
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(J1939_CLAIM_TIMEOUT_MS);
            }
        }

        if (!yielded) {
            /* 250 ms passed with no unresolved conflict */
            g_sa = sa;
            ESP_LOGI(TAG, "SA=0x%02X claimed (NAME=0x%016llX)", sa,
                     (unsigned long long)name);
            return;
        }

        /* Advance to next candidate address */
        sa++;
        if (sa > 0xFD) {
            sa = 0x00;
            wrapped = true;
        }
        /* Full circle with no success */
        if (wrapped && sa == start_sa) {
            ESP_LOGE(TAG, "No free address found – sending Cannot Claim (SA=0xFE)");
            send_addr_claimed(J1939_ADDR_NULL, name);
            g_sa = J1939_ADDR_NULL;
            return;
        }
    }
}

/* ── RX task ─────────────────────────────────────────────────────────────── */

void can_rx_task(void *arg)
{
    twai_message_t msg;

    while (1) {
        if (twai_receive(&msg, portMAX_DELAY) != ESP_OK) continue;

        /* Ignore Address Claimed frames in normal operation */
        if (msg.extd &&
            ((msg.identifier >> 16) & 0xFF) == 0xEE &&
            ((msg.identifier >>  8) & 0xFF) == 0xFF) {
            continue;
        }

        if (msg.extd) {
            //ESP_LOGI(TAG, "RX  ID=0x%08lX [ext] DLC=%d", (unsigned long)msg.identifier, msg.data_length_code);
            if(((msg.identifier >> 16) & 0xFF) == 0xFE &&
                ((msg.identifier >>  8) & 0xFF) == 0x45) { 
                CANPositionCommand = (int16_t)((msg.data[1] << 8) | msg.data[0]); /* store the position from the CAN message */
            }
        } else {
            
        }
        if (!msg.rtr) {
            for (int i = 0; i < msg.data_length_code; i++) {
                //printf(" %02X", msg.data[i]);
            }
            //printf("\n");
        }
        fail_count = 0;  /* reset TX fail count on any RX */
    }
}

/* ── TX task ─────────────────────────────────────────────────────────────── */

void can_tx_task(void *arg)
{

    while (1) {
        if (g_sa == J1939_ADDR_NULL || fail_count >= 6) {
            /* No address claimed – do not transmit */
            vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
            continue;
        }

        /*
         * Proprietary B broadcast frame (PGN 0xFF00, PDU2).
         * CAN ID = Priority(6) | PF(0xFF) | PS(0x00) | SA
         */
        uint32_t can_id = (6UL << 26) | (0xFEUL << 16) | (0x45UL << 8) | g_sa;

        twai_message_t msg = {
            .identifier       = can_id,
            .extd             = 1,
            .rtr              = 0,
            .ss               = 0,
            .self             = 0,
            .dlc_non_comp     = 0,
            .data_length_code = 8,
        };
        msg.data[0] = CANPositionCommand & 0xFF; /* send the current command for the motor */
        msg.data[1] = CANPositionCommand >> 8;
        msg.data[2] = CANPositionActual & 0xFF; /* send the current position of the motor */
        msg.data[3] = CANPositionActual >> 8;
        msg.data[4] = 0xFF;
        msg.data[5] = 0xFF;
        msg.data[6] = 0xFF;
        msg.data[7] = 0xFF;

        if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            fail_count=0;
        } else {
            ESP_LOGE(TAG, "TX failed");
            fail_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}

void can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO,
                                                                   TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 16;
    g_config.tx_queue_len = 8;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started at 250 kbps (TX=GPIO%d, RX=GPIO%d)", CAN_TX_GPIO, CAN_RX_GPIO);

    /* Derive a unique 21-bit identity number from the last 3 bytes of the MAC */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t identity = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    identity &= 0x1FFFFF;

    uint64_t name = j1939_build_name(identity);
    ESP_LOGI(TAG, "J1939 NAME = 0x%016llX  (identity=0x%05lX)",
             (unsigned long long)name, (unsigned long)identity);

    /* Run address claim – blocks until an address is secured (or fails) */
    j1939_address_claim(name);
}

int16_t can_get_PositionCommand(void) {
    return CANPositionCommand; /* return the current command for the motor */
}

void can_set_PositionActual(int16_t position) {
    CANPositionActual = position; /* set the current position of the motor */
}