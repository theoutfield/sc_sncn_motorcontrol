/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <rem_16mt_service.h>
#include <timer.h>
#include <print.h>
#include <xscope.h>
#include <mc_internal_constants.h>
#include <filters.h>

extern char start_message[];

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void init_spi_ports(SPIPorts &spi_ports)
{
    spi_master_init(spi_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
    slave_deselect(*spi_ports.slave_select); // Ensure slave select is in correct start state
}


int checksum_compute(unsigned count, unsigned singleturn_filtered, unsigned singleturn_raw, unsigned timestamp) {
    int computed_checksum = 0x5a ^ (1 + (timestamp & 0xff)) ^ (2 + (singleturn_raw & 0xff)) ^ (3 + (singleturn_raw >> 8)) ^ (4 + (singleturn_filtered & 0xff)) ^ (5 + (singleturn_filtered >> 8)) ^ (6 + (count & 0xff)) ^ (7 + (count >> 8));
    return computed_checksum & 0xff;
}

{ SensorError, int, unsigned int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports, UsecType tile_usec) {
    unsigned int timestamp;
    SensorError status;
    int count;
    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned int checksum;
    unsigned int computed_checksum;
    unsigned int try_count = 0;
    timer t;
    unsigned last_read;
    t :> last_read;
    last_read = last_read - 40*tile_usec - 1;

    do {
        t when timerafter(last_read + 40*tile_usec) :> void;
        configure_out_port(*spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1); //set mosi to 1
        slave_select(*spi_ports.slave_select);
        delay_ticks(10*tile_usec); //wait for the data buffer to fill
        count = spi_master_in_short(spi_ports.spi_interface);
        singleturn_filtered = spi_master_in_short(spi_ports.spi_interface);
        singleturn_raw = spi_master_in_short(spi_ports.spi_interface);
        timestamp = spi_master_in_byte(spi_ports.spi_interface);
        checksum = spi_master_in_byte(spi_ports.spi_interface);
        slave_deselect(*spi_ports.slave_select);
        t :> last_read;
        computed_checksum = checksum_compute(count, singleturn_filtered, singleturn_raw, timestamp);
        try_count++;
    } while(computed_checksum != checksum && try_count <= REM_16MT_MAX_RETRY);

    if (computed_checksum == checksum) {
        status = (count >> 12);    //REM 16MT error code
    } else {
        status = SENSOR_CHECKSUM_ERROR;
    }

    count = (sext(count & 0xfff, 12) * (1 << 16)) + singleturn_filtered; //convert multiturn to signed absolute count

    return { status, count, singleturn_filtered, singleturn_raw, timestamp };
}


void rem_16mt_write(SPIPorts &spi_ports, int opcode, int data, int data_bits, UsecType tile_usec)
{
    configure_out_port(*spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_select(*spi_ports.slave_select);
    delay_ticks(100*tile_usec);
    spi_master_out_byte(spi_ports.spi_interface, opcode);
    if (data_bits == 8) {
        spi_master_out_byte(spi_ports.spi_interface, data);
    } else if (data_bits == 16) {
        spi_master_out_short(spi_ports.spi_interface, data);
    } else if (data_bits == 32) {
        spi_master_out_word(spi_ports.spi_interface, data);
    }
    configure_out_port(*spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_deselect(*spi_ports.slave_select);
    delay_ticks(200020*tile_usec);
}

SensorError rem_16mt_init(SPIPorts &spi_ports, PositionFeedbackConfig &config)
{
    SensorError status;

    delay_ticks(100*config.tile_usec);
    //reset
    rem_16mt_write(spi_ports, REM_16MT_CTRL_RESET, 0, 0, config.tile_usec);
    //read status
    { status, void, void, void, void } = rem_16mt_read(spi_ports, config.tile_usec);
    delay_ticks(100*config.tile_usec);
    if (status != SENSOR_NO_ERROR)
        return status;
    //direction
    if (config.polarity == SENSOR_POLARITY_INVERTED)
        rem_16mt_write(spi_ports, REM_16MT_CONF_DIR, 1, 8, config.tile_usec);
    else
        rem_16mt_write(spi_ports, REM_16MT_CONF_DIR, 0, 8, config.tile_usec);
    //filter
    if (config.rem_16mt_config.filter == 1) {
        config.rem_16mt_config.filter = 0x02;
    } else if (config.rem_16mt_config.filter < 0) {
        config.rem_16mt_config.filter = 0x00;
    } else if (config.rem_16mt_config.filter > 9) {
        config.rem_16mt_config.filter = 0x09;
    }
    rem_16mt_write(spi_ports, REM_16MT_CONF_FILTER, config.rem_16mt_config.filter, 8, config.tile_usec);
    //read status
    { status, void, void, void, void } = rem_16mt_read(spi_ports, config.tile_usec);
    delay_ticks(100*config.tile_usec);
    return status;
}
