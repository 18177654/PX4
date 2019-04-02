/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * motor_current_mcp3428.cpp
 *
 *  Created on: 10 Oct 2018
 *      Author: anton
 *
 *  Driver for the MCP3428 A/D Converter measuring the currents of each motor.
 */
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_motor_current.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/motor_current.h>

#include <board_config.h>

/* Configuration Constants */
#define MCP3428_BUS PX4_I2C_BUS_EXPANSION
#define MCP3428_BASEADDR 0x68
#define MCP3428_DEVICE_PATH "/dev/mcp3428"

#define MCP_GENERAL_CALL_ADDR 0x00
#define MCP_CMD_RESET 0x06
#define MCP_REQ_READING 0x84

#define MCP_CALIBRATE 8.55
#define MCP_LSB 0.00025

#define ACS_V_PER_A 0.060

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

// MCP3428 class
class MCP3428 : public device::I2C {
public:
    MCP3428(int bus = MCP3428_BUS, int address = MCP3428_BASEADDR);
    virtual ~MCP3428();

    virtual int init();

    virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);
    virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

    /**
    * Diagnostics - print some basic information about the driver.
    */
    void print_info();

protected:
    virtual int probe();

private:
    bool _debug_enabled;
    int _conversion_interval;

    work_s              _work;
    ringbuffer::RingBuffer  *_reports;

    int _measure_ticks;

    int _class_instance;
    int _orb_class_instance;

    uint8_t _current_motor; // represents the current motor current begin read

    struct motor_current_s report;

    orb_advert_t        _motor_current_topic;

    perf_counter_t      _sample_perf;
    perf_counter_t      _comms_errors;

    /**
    * Initialise the automatic measurement state machine and start it.
    *
    * @note This function is called at open and error time.  It might make sense
    *       to make it more aggressive about resetting the bus in case of errors.
    */
    void                start();

    /**
    * Stop the automatic measurement state machine.
    */
    void                stop();

    /**
    * Perform a poll cycle; collect from the previous measurement
    * and start a new one.
    */
    void                cycle();
    int                 measure();
    int                 collect();

    /**
    * Static trampoline from the workq context; because we don't have a
    * generic workq wrapper yet.
    *
    * @param arg        Instance pointer for the driver that is polling.
    */
    static void         cycle_trampoline(void *arg);

    /**
     * Convert channel number to motor index.
     */
    int channel_to_motor(int ch);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int motor_current_mcp3428_main(int argc, char *argv[]);

MCP3428::MCP3428(int bus, int address) :
    I2C("MCP3428", MCP3428_DEVICE_PATH, bus, address, 100000),
    _reports(nullptr),
    _measure_ticks(0),
    _class_instance(-1),
    _orb_class_instance(-1),
    _motor_current_topic(nullptr),
    _sample_perf(perf_alloc(PC_ELAPSED, "mcp3428_read")),
    _comms_errors(perf_alloc(PC_COUNT, "mcp3428_com_err"))
{
    uint8_t i;

    // Enable debug() calls
    _debug_enabled = true;

    // Set conversion interval to 50 Hz
    _conversion_interval = 20000;

    // Set current motor to first motor
    _current_motor = 0;

    for(i = 0 ; i < 4 ; i++)
        report.motor_currents[i] = 0.0;

    // work_cancel in the dtor will explode if we don't do this...
    memset(&_work, 0, sizeof(_work));
}

MCP3428::~MCP3428()
{
    /* make sure we are truly inactive */
    stop();

    /* free any existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    // Unadvertise
    if (_motor_current_topic != nullptr) {
        orb_unadvertise(_motor_current_topic);
    }

    if (_class_instance != -1) {
        unregister_class_devname(MOTOR_CURRENT_BASE_DEVICE_PATH, _class_instance);
    }

    /* free perf counters */
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int MCP3428::init()
{
    uint8_t cmd;
    int ret = PX4_ERROR;

    // Do I2C init (and probe) first
    set_device_address(MCP3428_BASEADDR);
    if (I2C::init() != OK) {
        return ret;
    }

    // Allocate basic report buffers
    _reports = new ringbuffer::RingBuffer(2, sizeof(motor_current_s));

    if (_reports == nullptr) {
        return ret;
    }

    _class_instance = register_class_devname(MOTOR_CURRENT_BASE_DEVICE_PATH);

    /* get a publish handle on the motor currents topic */
    struct motor_current_s mc_report = {};

    _motor_current_topic = orb_advertise_multi(ORB_ID(motor_current), &mc_report, &_orb_class_instance, ORB_PRIO_HIGH);

    if (_motor_current_topic == nullptr) {
        DEVICE_LOG("failed to create motor_current object. Did you start uOrb?");
    }

    // Do a reset of the device.
    set_device_address(MCP_GENERAL_CALL_ADDR);
    cmd = MCP_CMD_RESET;
    ret = transfer(&cmd, 1, nullptr, 0);
    set_device_address(MCP3428_BASEADDR);

    if (OK != ret) {
        DEVICE_LOG("i2c::reset transfer returned %d", ret);
        return ret;
    }

    ret = OK;

    return ret;
}

int MCP3428::probe()
{
    return measure();
}

int MCP3428::ioctl(device::file_t *filp, int cmd, unsigned long arg) {
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {
            case SENSOR_POLLRATE_DEFAULT: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* set interval for next measurement to minimum legal value */
                    _measure_ticks = USEC2TICK(_conversion_interval);

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();

                    }

                    return OK;
                }

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* convert hz to tick interval via microseconds */
                    int ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(_conversion_interval)) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    _measure_ticks = ticks;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    default:
        /* give it to the superclass */
        return I2C::ioctl(filp, cmd, arg);
    }
}

ssize_t MCP3428::read(device::file_t *filp, char *buffer, size_t buflen) {
    unsigned count = buflen / sizeof(struct motor_current_s);
    struct motor_current_s *rbuf = reinterpret_cast<struct motor_current_s *>(buffer);
    int ret = 0;

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is enabled */
    if (_measure_ticks > 0) {

        /*
         * While there is space in the caller's buffer, and reports, copy them.
         * Note that we may be pre-empted by the workq thread while we are doing this;
         * we are careful to avoid racing with them.
         */
        while (count--) {
            if (_reports->get(rbuf)) {
                ret += sizeof(*rbuf);
                rbuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;
    }

    /* manual measurement - run one conversion */
    do {
        _reports->flush();

        /* trigger a measurement */
        if (OK != measure()) {
            ret = -EIO;
            break;
        }

        /* wait for it to complete */
        usleep(_conversion_interval);

        /* run the collection phase */
        if (OK != collect()) {
            ret = -EIO;
            break;
        }

        /* state machine will have generated a report, copy it out */
        if (_reports->get(rbuf)) {
            ret = sizeof(*rbuf);
        }

    } while (0);

    return ret;
}

int MCP3428::measure() {
    int ret;

    // Send the command to request a reading from the A/D of the current motor.
    uint8_t cmd = MCP_REQ_READING | ((_current_motor & 0x0F) << 5);
    set_device_address(MCP3428_BASEADDR); // make sure address is correct
    ret = transfer(&cmd, 1, nullptr, 0);

    if (OK != ret) {
        perf_count(_comms_errors);
        DEVICE_DEBUG("i2c::transfer returned %d", ret);
        return ret;
    }

    ret = OK;

    return ret;
}

int MCP3428::collect() {
    int ret = -EIO;
    perf_begin(_sample_perf);

    uint8_t val[3]; // value to populate when reading from device
    uint16_t current_reading; // current reading in bits
    bool isValid; // represents whether the reading is valid
    float current; // current motor current reading in Amps

    // Read from device
    ret = transfer(nullptr, 0, val, 3);

    if (ret < 0) {
        DEVICE_DEBUG("error reading from sensor: %d", ret);
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return ret;
    }

    // Convert reading and check whether reading is valid
    current_reading = ((val[0] & 0x3F) << 8) | val[1];
    isValid = !(val[2] >> 7);

    // Only store the reading and move on to the next motor if the reading is valid.
    if(isValid) {
        // Convert reading to motor current Amps
        current = ((float(current_reading) * float(MCP_LSB)) / float(ACS_V_PER_A)) - float(MCP_CALIBRATE);

        // Store reading
        report.motor_currents[channel_to_motor(_current_motor)] = current;

        // Read next motor on next execution
        _current_motor++;
        if(_current_motor > 3) {
            _current_motor = 0;
        }
    }

    // Publish it
    if (_motor_current_topic != nullptr) {
        report.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(motor_current), _motor_current_topic, &report);
    }

    _reports->force(&report);

    // Notify anyone waiting for data
    poll_notify(POLLIN);

    ret = OK;

    perf_end(_sample_perf);
    return ret;
}

void MCP3428::start() {
    /* reset the report ring and state machine */
    _reports->flush();

    /* request a measurement */
    measure();

    /* schedule a cycle to start things */
    work_queue(HPWORK, &_work, (worker_t)&MCP3428::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void MCP3428::stop() {
    work_cancel(HPWORK, &_work);
}

void MCP3428::cycle_trampoline(void *arg) {
    MCP3428 *dev = (MCP3428 *)arg;

    dev->cycle();
}

void MCP3428::cycle() {
    // Collect results
    if (OK != collect()) {
        DEVICE_DEBUG("collection error");
        // If error restart the measurement state machine
        start();
        return;
    }

    /* request a measurement */
    measure();

    // Schedule a fresh cycle call when the measurement is done
    work_queue(HPWORK, &_work, (worker_t)&MCP3428::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void MCP3428::print_info()
{
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
    printf("poll interval:  %u ticks\n", _measure_ticks);
    _reports->print_info("report queue");
}


int MCP3428::channel_to_motor(int ch)
{
    int motor = 0;
    switch(ch)
    {
    case 0:
        motor = 3;
        break;
    case 1:
        motor = 0;
        break;
    case 2:
        motor = 2;
        break;
    case 3:
        motor = 1;
        break;
    }

    return motor;
}

/**
 * Local functions in support of the shell command.
 */
namespace motor_current_mcp3428 {

MCP3428 *g_dev;

void    start();
void    stop();
void    test();
void    reset();
void    info();

void start() {
    int fd = -1;

    if (g_dev != nullptr) {
        errx(1, "already started");
    }

    /* create the driver */
    g_dev = new MCP3428();

    if (g_dev == nullptr) {
        goto fail;
    }

    if (OK != g_dev->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(MCP3428_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        ::close(fd);
        goto fail;
    }

    ::close(fd);
    exit(0);

fail:

    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;
    }

    errx(1, "driver start failed");
}

void    stop() {
    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;

    } else {
        errx(1, "driver not running");
    }

    exit(0);
}

void test() {
    struct motor_current_s report;
    ssize_t sz;
    int ret;

    int fd = open(MCP3428_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        err(1, "%s open failed (try 'mcp3428 start' if the driver is not running", MCP3428_DEVICE_PATH);
    }

    /* do a simple demand read */
    sz = read(fd, &report, sizeof(report));

    if (sz != sizeof(report)) {
        err(1, "immediate read failed");
    }

    print_message(report);

    /* start the sensor polling at 2Hz */
    if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
        errx(1, "failed to set 2Hz poll rate");
    }

    /* read the sensor 12x and report each value */
    for (unsigned i = 0; i < 12; i++) {
        struct pollfd fds;

        /* wait for data to be ready */
        fds.fd = fd;
        fds.events = POLLIN;
        ret = poll(&fds, 1, 2000);

        if (ret != 1) {
            errx(1, "timed out waiting for sensor data");
        }

        /* now go get it */
        sz = read(fd, &report, sizeof(report));

        if (sz != sizeof(report)) {
            err(1, "periodic read failed");
        }

        print_message(report);
    }

    /* reset the sensor polling to default rate */
    if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
        errx(1, "failed to set default poll rate");
    }

    ::close(fd);
    errx(0, "PASS");
}

void reset() {
    int fd = open(MCP3428_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "driver poll restart failed");
    }

    ::close(fd);
    exit(0);
}

void info() {
    if (g_dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("state @ %p\n", g_dev);
    g_dev->print_info();

    exit(0);
}

} /* namespace */

int motor_current_mcp3428_main(int argc, char *argv[])
{
    // Check number of arguments.
    if (argc < 2) {
        PX4_WARN("usage: <start/stop>");
        return PX4_ERROR;
    }

    // Check argument - start / stop / test / reset / info
    if (!strcmp(argv[1], "start")) {
        motor_current_mcp3428::start();
    } else if (!strcmp(argv[1], "stop")) {
        motor_current_mcp3428::stop();
    } else if (!strcmp(argv[1], "test")) {
        motor_current_mcp3428::test();
    }else if (!strcmp(argv[1], "reset")) {
        motor_current_mcp3428::reset();
    } else if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
        motor_current_mcp3428::info();
    } else {
        PX4_WARN("action (%s) not supported", argv[1]);
        return PX4_ERROR;
    }

    return PX4_OK;
}

