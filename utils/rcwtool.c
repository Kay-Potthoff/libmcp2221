/* -*-C-*- */
/* SPDX-License-Identifier:    GPL-3.0 */
/*
 * Copyright (C) 2020 MicroSys Electronics GmbH
 * Author: Kay Potthoff <kay.potthoff@microsys.de>
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <inttypes.h>
#include <endian.h>
#include <stdbool.h>

#include "libmcp2221/libmcp2221.h"

#define DEFAULT_EEPROM_ADDRESS 0x50

#define MAX_BUFSZ 60

#define IDX_HELP 0
#define IDX_ADDR 1
#define IDX_BOOT 2

#define WAIT_PERIOD_50MS 7
#define RCON_SRC_I2C 1

typedef enum {
    BOOT_QSPI = 0,
    BOOT_SD = 2,
    BOOT_MMC = 3
} boot_t;

typedef union rcw_u {
    uint8_t data[4];
    uint32_t boot_cfg;
    struct {
        uint32_t phy:2;
        uint32_t res0:3;
        uint32_t boot:3;
        uint32_t src:1;
        uint32_t res1:6;
        uint32_t xosc:1;
        uint32_t res2:3;
        uint32_t res3:12;
        uint32_t pll:1;
    } rcon;
    struct {
        uint32_t phy:2;
        uint32_t res0:3;
        uint32_t boot:3;
        uint32_t src:1;
        uint32_t res1:6;
        uint32_t xosc:1;
        uint32_t wait:3;
        uint32_t speed:1;
        uint32_t res2:11;
        uint32_t pll:1;
    } sd;
    struct {
        uint32_t phy:2;
        uint32_t res0:3;
        uint32_t boot:3;
        uint32_t src:1;
        uint32_t res1:6;
        uint32_t xosc:1;
        uint32_t wait:3;
        uint32_t mode:4;
        uint32_t res2:8;
        uint32_t pll:1;
    } mmc;
    struct {
        uint32_t phy:2;
        uint32_t mode:3;
        uint32_t boot:3;
        uint32_t src:1;
        uint32_t port:1;
        uint32_t ck2:1; // CK2 clock
        uint32_t cas:4; // QuadSPI_SFACR[CAS] value
        uint32_t xosc:1;
        uint32_t por_delay:3;
        uint32_t ckn:1; // differential clock
        uint32_t res3:2;
        uint32_t tdh:2; // time hold delay
        uint32_t fsphs:1; // full speed phase selection SMPR[FSPHS]
        uint32_t fsdly:1; // full speed delay selection SMPR[FSDLY]
        uint32_t dllfsmpf:3; // selects the n'th tap
        uint32_t dqs_sel:2;
        uint32_t pll:1;
    } qspi;
} rcw_t;

static const char *const short_options = "h";

static const struct option long_options[] = {
        [IDX_HELP] = {"help",        no_argument, 0, 0},
        [IDX_ADDR] = {"addr",        required_argument, 0, 0},
        [IDX_BOOT] = {"boot",        required_argument, 0, 0},
        // end of list
        {0, 0, 0, 0}
};

static void print_help()
{
    puts("mcp_eeprom: read/write EEPROM connected to I2C");
    puts("usage: mcp_eeprom [options]");
    puts("    -h|--help                 print this help\n");
    puts("    --addr=<address>          7-bit I2C address (hex)\n");
    puts("    --boot=<qspi,sd,mmc>      define boot media\n");
}

static int at24c01_read(mcp2221_t *dev,
        const int address,
        const uint8_t offset,
        uint8_t *const r_buf,
        const unsigned int r_len)
{
    return mcp2221_i2cWriteRead(dev, address,
            &offset, 1,
            r_buf, r_len);
}

static int at24c01_write(mcp2221_t *dev,
        const int address,
        const uint8_t offset,
        const uint8_t *const w_buf,
        const unsigned int w_len)
{
    uint8_t wbuf[MAX_BUFSZ+1];

    wbuf[0] = offset;
    memcpy(&wbuf[1], w_buf, w_len);

    return mcp2221_i2cWriteRead(dev, address,
            wbuf, w_len+1,
            NULL, 0);
}

static int s32g_rcw_read(mcp2221_t *dev,
        const int address,
        rcw_t *const rcw)
{
    int res = at24c01_read(dev, address, 0, rcw->data, sizeof(*rcw));

    rcw->boot_cfg = le32toh(rcw->boot_cfg);

    return res;
}

static int s32g_rcw_write(mcp2221_t *dev,
        const int address,
        const rcw_t *const rcw)
{
    rcw_t r = *rcw;

    r.boot_cfg = htole32(r.boot_cfg);

    return at24c01_write(dev, address, 0, r.data, sizeof(r));
}

static unsigned int wait_period_to_ms(unsigned int p)
{
    switch (p) {
    case 1: return 5;
    case 2: return 10;
    case 3: return 20;
    case 4: return 35;
    case WAIT_PERIOD_50MS: return 50;
    case 0:
    default:
        return 0;
    }
}

static const char *const boot_to_str(unsigned int boot)
{
    switch (boot) {
    case 0: return "QSPI";
    case 2: return "SD";
    case 3: return "MMC";
    default: return "?";
    }
}

static const char *const src_to_str(unsigned int src)
{
    switch (src) {
    case 0: return "parallel";
    case RCON_SRC_I2C: return "I2C";
    default: return "?";
    }
}

static const char *const xosc_to_str(unsigned int xosc)
{
    switch (xosc) {
    case 0: return "differential/crystal";
    case 1: return "bypass";
    default: return "?";
    }
}

static const char *const pll_to_str(unsigned int pll)
{
    switch (pll) {
    case 0: return "PLL@IRC";
    case 1: return "IRC@48MHz";
    default: return "?";
    }
}

static const char *const phy_to_str(unsigned int phy)
{
    switch (phy) {
    case 0: return "RMII";
    case 1: return "SGMII";
    case 2: return "RGMII";
    case 3: return "No PHY";
    default: return "?";
    }
}

static void print_rcw(const rcw_t *const rcw)
{
      puts("RCW");
      puts("=================================");
    printf("    PHY:   %s\n", phy_to_str(rcw->rcon.phy));
    printf("    BOOT:  %s\n", boot_to_str(rcw->rcon.boot));
    printf("    SRC:   %s\n", src_to_str(rcw->rcon.src));
    printf("    XOSC:  %s\n", xosc_to_str(rcw->rcon.xosc));
    printf("    PLL:   %s\n", pll_to_str(rcw->rcon.pll));
    if (rcw->rcon.boot == BOOT_SD) {
        printf("    WAIT:  %dms\n", wait_period_to_ms(rcw->sd.wait));
        printf("    SPEED: %s\n", rcw->sd.speed ? "high":"default");
    }
}

int main(int argc, char **argv)
{
    int opt;
    int option_index = 0;
    boot_t boot_media = BOOT_SD;
    int eeprom_address = DEFAULT_EEPROM_ADDRESS;

    mcp2221_t *dev = NULL;

    puts("rcwtool: read/write I2C-EEPROM");


    while ((opt = getopt_long(argc, argv, short_options,
            long_options, &option_index)) != -1) {

        switch (opt) {
        case 0:
        {
            switch (option_index) {
            case IDX_HELP:
                print_help();
                return 0;
            case IDX_ADDR:
                sscanf(optarg, "%x", &eeprom_address);
                break;
            case IDX_BOOT:
                if (strcmp(optarg, "qspi")==0)
                    boot_media = BOOT_QSPI;
                else if (strcmp(optarg, "sd")==0)
                    boot_media = BOOT_SD;
                else if (strcmp(optarg, "mmc")==0)
                    boot_media = BOOT_MMC;
                break;
            }
        }
        break;
        case 'h':
            print_help();
            return 0;
        }
    }

    if (eeprom_address < 0 || eeprom_address > 0x7f) {
        fprintf(stderr, "Error: illegal address: 0x%02x!\n", eeprom_address);
        return -1;
    }

    mcp2221_init();

    const int count = mcp2221_find(MCP2221_DEFAULT_VID,
            MCP2221_DEFAULT_PID,
            NULL, NULL, NULL);

    if (count == 0) {
        puts("Note: no devices found!");
        mcp2221_exit();
        return -1;
    }

    printf("Found %d device%c\n", count, count==1?' ':'s');

    if (count == 1)
        dev = mcp2221_open();
    else {
        int num = 0;
        printf("Found %d devices\n", count);
        printf("Enter number of desired device [0-%d]: ", count-1);
        scanf("%d", &num);
        if (num < 0 || num >= count) {
            fprintf(stderr, "Error: illegal device number out of range!");
            mcp2221_exit();
            return -1;
        }
        dev = mcp2221_open_byIndex(num);
    }

    if (!dev) {
        fprintf(stderr, "Error: cannot open MCP2221 device: %s\n",
                strerror(errno));
        mcp2221_exit();
        return -1;
    }

    mcp2221_error res = MCP2221_SUCCESS;
    mcp2221_i2c_state_t state = MCP2221_I2C_IDLE;

    rcw_t rcw;
    bool changed;

    res = mcp2221_i2cState(dev, &state);
    if (res != MCP2221_SUCCESS) {
        fprintf(stderr, "Error: cannot get state!\n");
        mcp2221_exit();
        return -1;
    }

    if (state != MCP2221_I2C_IDLE)
        mcp2221_i2cCancel(dev);

    // ucI2cDiv = (u8)((12000000/frequency) - 3);
    // 27 => 400kHz
    // 117 => 100kHz

    res = mcp2221_i2cDivider(dev, 27);
    if (res != MCP2221_SUCCESS) {
        fprintf(stderr, "Error: cannot set divider!\n");
        mcp2221_exit();
        return -1;
    }

    res = s32g_rcw_read(dev, eeprom_address, &rcw);

    if (res != MCP2221_SUCCESS) {
        fprintf(stderr, "Error: cannot read RCW: rv=%d!\n", res);
        mcp2221_exit();
        return -1;
    }

    printf("RCW: 0x%08x\n", rcw.boot_cfg);
    print_rcw(&rcw);

    if (rcw.rcon.src != RCON_SRC_I2C) {
        rcw.rcon.src = RCON_SRC_I2C;
        changed = true;
    }

    if (rcw.rcon.boot != boot_media) {
        rcw.boot_cfg = 0;
        rcw.rcon.boot = boot_media;
        changed = true;
    }

    if (rcw.rcon.boot == BOOT_SD) {

        if (rcw.sd.speed == 0) {
            rcw.sd.speed = 1;
            changed = true;
        }

        if (rcw.sd.wait != WAIT_PERIOD_50MS) {
            rcw.sd.wait = WAIT_PERIOD_50MS;
            changed = true;
        }
    }

    if (changed) {

        res = s32g_rcw_write(dev, eeprom_address, &rcw);

        if (res != MCP2221_SUCCESS) {
            fprintf(stderr, "Error: cannot write RCW: rv=%d!\n", res);
            mcp2221_exit();
            return -1;
        }

        res = s32g_rcw_read(dev, eeprom_address, &rcw);

        if (res != MCP2221_SUCCESS) {
            fprintf(stderr, "Error: cannot read RCW: rv=%d!\n", res);
            mcp2221_exit();
            return -1;
        }

        printf("RCW: 0x%08x\n", rcw.boot_cfg);
        print_rcw(&rcw);
    }

    mcp2221_exit();

    return 0;
}

/* *INDENT-OFF* */
/******************************************************************************
 * Local Variables:
 * mode: C
 * c-indent-level: 4
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 * kate: space-indent on; indent-width 4; mixedindent off; indent-mode cstyle;
 * vim: set expandtab filetype=c:
 * vi: set et tabstop=4 shiftwidth=4: */
