/*
Copyright (c) 2016, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <Wire.h>
#include "RTCTimer.h"
#include "RTCZero.h"
#include "LoRaHelper.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "BootMenu.h"
#include "ublox.h"
#include "MyTime.h"
#include "ReportDataRecord.h"
#include "GpsFixDataRecord.h"
#include "OverTheAirConfigDataRecord.h"
#include "GpsFixLiFoRingBuffer.h"
#include "LSM303AGR.h"
#include "LedColor.h"
#include "Enums.h"
#include "CayenneLPP.h"

//#define DEBUG

#define PROJECT_NAME "SodaqOne Universal Tracker v2"
#define VERSION "5.0.2"
#define STARTUP_DELAY 5000

// #define DEFAULT_TEMPERATURE_SENSOR_OFFSET 33
// #define DEFAULT_LORA_PORT 2
// #define DEFAULT_IS_OTAA_ENABLED 1
// #define DEFAULT_DEVADDR_OR_DEVEUI "0000000000000000"
// #define DEFAULT_APPSKEY_OR_APPEUI "00000000000000000000000000000000"
// #define DEFAULT_NWSKEY_OR_APPKEY "00000000000000000000000000000000"

#define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK
#define GPS_COMM_CHECK_TIMEOUT 3 // seconds

#define MAX_RTC_EPOCH_OFFSET 25

#define ADC_AREF 3.3f
#define BATVOLT_R1 4.7f
#define BATVOLT_R2 10.0f

#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB
#define LORA_STREAM Serial1

#define CONSOLE_BAUD 115200
#define DEBUG_BAUD 115200 // only used when CONSOLE is different than debug, otherwise console baud is used only

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

// macro to do compile time sanity checks / assertions
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#define consolePrint(x) CONSOLE_STREAM.print(x)
#define consolePrintln(x) CONSOLE_STREAM.println(x)

#define debugPrint(x) if (params.getIsDebugOn()) { DEBUG_STREAM.print(x); }
#define debugPrintln(x) if (params.getIsDebugOn()) { DEBUG_STREAM.println(x); }

#ifndef LORA_RESET
#define LORA_RESET -1
#endif

RTCZero rtc;
RTCTimer timer;
UBlox ublox;
Time time;
LSM303AGR accelerometer;

ReportDataRecord pendingReportDataRecord;
bool isPendingReportDataRecordNew; // this is set to true only when pendingReportDataRecord is written by the delegate

volatile bool minuteFlag;
volatile bool isOnTheMoveActivated;
volatile uint32_t lastOnTheMoveActivationTimestamp;
volatile bool updateOnTheMoveTimestampFlag;

static uint8_t lastResetCause;
static bool isGpsInitialized;
static bool isRtcInitialized;
static bool isDeviceInitialized;
static bool isOnTheMoveInitialized;
static int64_t rtcEpochDelta; // set in setNow() and used in getGpsFixAndTransmit() for correcting time in loop

static uint8_t loraHWEui[8];
static bool isLoraHWEuiInitialized;

CayenneLPP cayenneRecord(51);

void setup();
void loop();

uint32_t getNow();
void setNow(uint32_t now);
void handleBootUpCommands();
void initRtc();
void accelerometerInt1Handler();
void rtcAlarmHandler();
void initRtcTimer();
void resetRtcTimerEvents();
void initSleep();
bool initLora(LoraInitConsoleMessages messages, LoraInitJoin join);
bool initGps();
void initOnTheMove();
void systemSleep();
void runDefaultFixEvent(uint32_t now);
void runAlternativeFixEvent(uint32_t now);
void runLoraModuleSleepExtendEvent(uint32_t now);
void setGpsActive(bool on);
void setAccelerometerTempSensorActive(bool on);
bool isAlternativeFixEventApplicable();
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo);
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
bool getGpsFixAndTransmit();
uint8_t getBatteryVoltage();
int8_t getBoardTemperature();
void updateConfigOverTheAir(const uint8_t* buffer, uint16_t size);
void getHWEUI();
void setDevAddrOrEUItoHWEUI();
void onConfigReset(void);
void setupBOD33();

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);

void setup()
{
    lastResetCause = PM->RCAUSE.reg;

    // In case of reset (this is probably unnecessary)
    sodaq_wdt_disable();
  
    // Setup the BOD33
    setupBOD33();
    
    sodaq_wdt_enable(WDT_PERIOD_8X);
    sodaq_wdt_reset();

    CONSOLE_STREAM.begin(CONSOLE_BAUD);
    if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    setLedColor(RED);
    sodaq_wdt_safe_delay(STARTUP_DELAY);
    printBootUpMessage(CONSOLE_STREAM);

    gpsFixLiFoRingBuffer_init();
    initSleep();
    initRtc();

    Wire.begin();

    // init params
    params.setConfigResetCallback(onConfigReset);
    params.read();

    // if allowed, init early for faster initial fix
    if (params.getIsGpsOn()) {
        isGpsInitialized = initGps();
    }

    // disable the watchdog only for the boot menu
    sodaq_wdt_disable();
    handleBootUpCommands();
    sodaq_wdt_enable(WDT_PERIOD_8X);

    // make sure the debug option is honored
    if (params.getIsDebugOn() && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    // make sure the GPS status honors the new user params
    if (!params.getIsGpsOn()) {
        isGpsInitialized = false; // force not to use GPS
    }
    else if (!isGpsInitialized) {
        isGpsInitialized = initGps();
    }

    initLora(LORA_INIT_SHOW_CONSOLE_MESSAGES, LORA_INIT_JOIN);

    accelerometer.disableMagnetometer();
    if (params.getAccelerationPercentage() > 0) {
        initOnTheMove();

        isOnTheMoveInitialized = true;
    }

    initRtcTimer();

    isDeviceInitialized = true;

    consolePrintln("** Boot-up completed successfully!");
    sodaq_wdt_reset();

    // disable the USB if not needed for debugging
    if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        consolePrintln("The USB is going to be disabled now.");
        debugPrintln("The USB is going to be disabled now.");

        SerialUSB.flush();
        sodaq_wdt_safe_delay(3000);
        SerialUSB.end();
        USBDevice.detach();
        USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
    }

    // disable the debug stream if it is not disabled by the above
    if (!params.getIsDebugOn() && ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        DEBUG_STREAM.flush();
        DEBUG_STREAM.end();
    }

    // disable the console stream if it is not disabled by the above, 
    // and only if it is different than the debug stream
    if ((long)&CONSOLE_STREAM != (long)&SerialUSB && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        CONSOLE_STREAM.flush();
        CONSOLE_STREAM.end();
    }

    if (getGpsFixAndTransmit()) {
        setLedColor(GREEN);
        sodaq_wdt_safe_delay(800);
    }
}

void loop()
{
    if (sodaq_wdt_flag) {
        // Reset watchdog
        sodaq_wdt_reset();
        sodaq_wdt_flag = false;

        LoRa.loopHandler();
    }

    if (updateOnTheMoveTimestampFlag) {
        lastOnTheMoveActivationTimestamp = getNow();
        updateOnTheMoveTimestampFlag = false;
    }

    if (minuteFlag) {
        if (params.getIsLedEnabled()) {
            setLedColor(BLUE);
        }

        timer.update(); // handle scheduled events

        minuteFlag = false;
    }

    systemSleep();
}

/**
 * Initializes the CPU sleep mode.
 */
void initSleep()
{
    // Set the sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * Returns the battery voltage minus 3 volts.
 */
uint8_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BAT_VOLT));
    voltage = (voltage - 3000) / 10;

    return (voltage > 255 ? 255 : (uint8_t)voltage);
}

/**
 * Returns the board temperature.
*/
int8_t getBoardTemperature()
{
    setAccelerometerTempSensorActive(true);

    int8_t temp = params.getTemperatureSensorOffset() + accelerometer.getTemperatureDelta();

    setAccelerometerTempSensorActive(false);

    return temp;
}

/**
 * Transmits the current data (using the current "pendingReportDataRecord").
 */
void transmit()
{
    if (params.getIsCayennePayloadEnabled()) {
        // Reset the record
        cayenneRecord.reset();

        // Add GPS record on data channel 1
        float latitude = (float)pendingReportDataRecord.getLat() / 10000000.0f;
        float longitude = (float)pendingReportDataRecord.getLong() / 10000000.0f;
        float altitude = (float)pendingReportDataRecord.getAltitude();
        cayenneRecord.addGPS(1, latitude, longitude, altitude);

        // Add battery voltage on data channel 2
        float voltage = 3.0 + 0.01 * pendingReportDataRecord.getBatteryVoltage();
        cayenneRecord.addAnalogInput(2, voltage);

        // Add temperature on data channel 3
        float temp = (float)pendingReportDataRecord.getBoardTemperature();
        cayenneRecord.addTemperature(3, temp);

        // Copy out the formatted record
        LoRa.transmit(cayenneRecord.getBuffer(), cayenneRecord.getSize());
    }
    else {
        uint8_t sendBuffer[51];
        uint8_t sendBufferSize = 0;

        // copy the pendingReportDataRecord into the sendBuffer
        memcpy(sendBuffer, pendingReportDataRecord.getBuffer(), pendingReportDataRecord.getSize());
        sendBufferSize = pendingReportDataRecord.getSize();

        // copy the previous coordinates if applicable (-1 because one coordinate is already in the report record)
        GpsFixDataRecord record;
        for (uint8_t i = 0; i < params.getCoordinateUploadCount() - 1; i++) {
            record.init();

            // (skip first record because it is in the report record already)
            if (!gpsFixLiFoRingBuffer_peek(1 + i, &record)) {
                break;
            }

            if (!record.isValid()) {
                break;
            }

            record.updatePreviousFixValue(pendingReportDataRecord.getTimestamp());
            memcpy(&sendBuffer[sendBufferSize - 1], record.getBuffer(), record.getSize());
            sendBufferSize += record.getSize();
        }

        LoRa.transmit(sendBuffer, sendBufferSize);
    }
}

/**
* Uses the "receiveBuffer" (received from LoRaWAN) to update the configuration.
*/
void updateConfigOverTheAir(const uint8_t* buffer, uint16_t size)
{
    OverTheAirConfigDataRecord record;
    record.init();
    record.copyFrom(buffer, size);

    if (record.isValid()) {
        params._defaultFixInterval = record.getDefaultFixInterval();
        params._alternativeFixInterval = record.getAlternativeFixInterval();

        // time of day seconds assumed
        params._alternativeFixFromHours = record.getAlternativeFixFrom() / 3600;
        params._alternativeFixFromMinutes = (record.getAlternativeFixFrom() - params._alternativeFixFromHours * 3600) / 60;

        params._alternativeFixToHours = record.getAlternativeFixTo() / 3600;
        params._alternativeFixToMinutes = (record.getAlternativeFixTo() - params._alternativeFixToHours * 3600) / 60;

        params._gpsFixTimeout = record.getGpsFixTimeout();

        params.commit(true);
        debugPrintln("OTAA Config commited!");

        // apply the rtc timer changes
        resetRtcTimerEvents();
    }
    else {
        debugPrintln("OTAA Config record is not valid!");
    }
}

/**
    Initializes the lora module according to the given operation (join or skip).
    Returns true if the operation was successful.
*/
bool initLora(LoraInitConsoleMessages messages, LoraInitJoin join)
{
    debugPrintln("Initializing LoRa...");

    if (messages == LORA_INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing LoRa...");
    }

    if (params.getIsDebugOn()) {
        LoRaBee.setDiag(DEBUG_STREAM);
        LoRa.setDiag(DEBUG_STREAM);
    }

    bool result;

    LORA_STREAM.begin(LoRaBee.getDefaultBaudRate());
    result = LoRaBee.init(LORA_STREAM, LORA_RESET);
    LoRaBee.setReceiveCallback(updateConfigOverTheAir);
    LoRa.init(LoRaBee, getNow);

    // set keys and parameters
    LoRa.setKeys(params.getDevAddrOrEUI(), params.getAppSKeyOrEUI(), params.getNwSKeyOrAppKey());
    LoRa.setOtaaOn(params.getIsOtaaEnabled());
    LoRa.setAdrOn(params.getIsAdrOn());
    LoRa.setAckOn(params.getIsAckOn());
    LoRa.setReconnectOnTransmissionOn(params.getShouldRetryConnectionOnSend());
    LoRa.setDefaultLoRaPort(params.getLoraPort());
    LoRa.setRepeatTransmissionCount(params.getRepeatCount());
    LoRa.setSpreadingFactor(params.getSpreadingFactor());
    LoRa.setPowerIndex(params.getPowerIndex());

    if (join == LORA_INIT_JOIN) {
        result = LoRa.join();

        if (messages == LORA_INIT_SHOW_CONSOLE_MESSAGES) {
            if (result) {
                consolePrintln("LoRa initialized.");
            }
            else {
                consolePrintln("LoRa initialization failed!");
            }
        }
    }

    LoRa.setActive(false); // make sure it is off
    return result;
}

/**
 * Initializes the GPS and leaves it on if succesful.
 * Returns true if successful.
*/
bool initGps()
{
    pinMode(GPS_ENABLE, OUTPUT);
    pinMode(GPS_TIMEPULSE, INPUT);

    // attempt to turn on and communicate with the GPS
    ublox.enable();
    ublox.flush();

    uint32_t startTime = getNow();
    bool found = false;
    while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
        sodaq_wdt_reset();

        found = ublox.exists();
    }

    // check for success
    if (found) {
        setGpsActive(true); // properly turn on before returning

        return true;
    }

    consolePrintln("*** GPS not found!");
    debugPrintln("*** GPS not found!");

    // turn off before returning in case of failure
    setGpsActive(false);

    return false;
}

/**
* Initializes the on-the-move functionality (interrupt on acceleration).
*/
void initOnTheMove()
{
    pinMode(ACCEL_INT1, INPUT);
    attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

    // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;

    accelerometer.enableAccelerometer(
        LSM303AGR::LowPowerMode,
        LSM303AGR::HrNormalLowPower10Hz, 
        LSM303AGR::XYZ,
        LSM303AGR::Scale8g,
        true);
    sodaq_wdt_safe_delay(100);

    accelerometer.enableInterrupt1(
        LSM303AGR::XHigh | LSM303AGR::XLow | LSM303AGR::YHigh | LSM303AGR::YLow | LSM303AGR::ZHigh | LSM303AGR::ZLow,
        params.getAccelerationPercentage() * 8.0 / 100.0,
        params.getAccelerationDuration(),
        LSM303AGR::MovementRecognition);
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    LORA_STREAM.flush();

    setLedColor(NONE);
    setGpsActive(false); // explicitly disable after resetting the pins

    // go to sleep, unless USB is used for debugging
    if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        noInterrupts();
        if (!(sodaq_wdt_flag || minuteFlag)) {
            interrupts();

            __WFI(); // SAMD sleep
        }
        interrupts();
    }
}

/**
 * Setup BOD33
 *
 * Setup BOD the way we want it.
 *  - BOD33USERLEVEL = 0x30 - shutdown at 3.07 Volt
 *  - BOD33_EN = [X] //Enabled
 *  - BOD33_ACTION = 0x01
 *  - BOD33_HYST = [X] //Enabled
 */
void setupBOD33()
{
    SYSCTRL->BOD33.bit.LEVEL = 0x30;    // 3.07 Volt
    SYSCTRL->BOD33.bit.ACTION = 1;      // Go to Reset
    SYSCTRL->BOD33.bit.ENABLE = 1;      // Enabled
    SYSCTRL->BOD33.bit.HYST = 1;        // Hysteresis on
    while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
        /* Wait for synchronization */
    }
}

/**
 * Returns the current datetime (seconds since unix epoch).
 */
uint32_t getNow()
{
    return rtc.getEpoch();
}

/**
 * Sets the RTC epoch and "rtcEpochDelta".
 */
void setNow(uint32_t newEpoch)
{
    uint32_t currentEpoch = getNow();

    debugPrint("Setting RTC from ");
    debugPrint(currentEpoch);
    debugPrint(" to ");
    debugPrintln(newEpoch);

    rtcEpochDelta = newEpoch - currentEpoch;
    rtc.setEpoch(newEpoch);

    timer.adjust(currentEpoch, newEpoch);

    isRtcInitialized = true;
}

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
    setResetDevAddrOrEUItoHWEUICallback(setDevAddrOrEUItoHWEUI);

    do {
        showBootMenu(CONSOLE_STREAM);
    } while (!params.checkConfig(CONSOLE_STREAM));

    params.showConfig(&CONSOLE_STREAM);
    params.commit();
}

/**
 * Initializes the RTC.
 */
void initRtc()
{
    rtc.begin();

    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

    // Attach handler
    rtc.attachInterrupt(rtcAlarmHandler);

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
}

/**
 * Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler()
{
    minuteFlag = true;
}

/**
 * Runs every time acceleration is over the limits
 * set by the user (if enabled).
*/
void accelerometerInt1Handler()
{
    if (digitalRead(ACCEL_INT1)) {
        if (params.getIsLedEnabled()) {
            setLedColor(YELLOW);
        }

        // debugPrintln("On-the-move is triggered");

        isOnTheMoveActivated = true;
        updateOnTheMoveTimestampFlag = true;
    }
}

/**
 * Initializes the RTC Timer and schedules the default events.
 */
void initRtcTimer()
{
    timer.setNowCallback(getNow); // set how to get the current time
    timer.allowMultipleEvents();

    resetRtcTimerEvents();
}

/**
 * Clears the RTC Timer events and schedules the default events.
 */
void resetRtcTimerEvents()
{
    timer.clearAllEvents();

    // Schedule the default fix event (if applicable)
    if (params.getDefaultFixInterval() > 0) {
        timer.every(params.getDefaultFixInterval() * 60, runDefaultFixEvent);
    }

    // check if the alternative fix event should be scheduled at all
    if (params.getAlternativeFixInterval() > 0) {
        // Schedule the alternative fix event
        timer.every(params.getAlternativeFixInterval() * 60, runAlternativeFixEvent);
    }

    if (isOnTheMoveInitialized) {
        timer.every(params.getOnTheMoveFixInterval() * 60, runOnTheMoveFixEvent);
    }

    // if lora is not enabled, schedule an event that takes care of extending the sleep time of the module
    if (!LoRa.isInitialized()) {
        timer.every(24 * 60 * 60, runLoraModuleSleepExtendEvent); // once a day
    }
}

/**
 * Returns true if the alternative fix event should run at the current time.
*/
bool isAlternativeFixEventApplicable()
{
    // - RTC should be initialized (synced time)
    // - alternative fix interval should be set
    // - the span between FROM and TO should be at least as much as the alternative fix interval
    // - current time should be within the FROM and TO times set
    return (isRtcInitialized
        && (params.getAlternativeFixInterval() > 0)
        && (params.getAlternativeFixTo() - params.getAlternativeFixFrom() >= params.getAlternativeFixInterval() * 60)
        && (isCurrentTimeOfDayWithin(params.getAlternativeFixFrom(), params.getAlternativeFixTo())));
}

/**
 * Returns true if the current rtc time is within the given times of day (in seconds).
*/
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo)
{
    uint32_t daySecondsCurrent = rtc.getHours() * 60 * 60 + rtc.getMinutes() * 60;

    return (daySecondsCurrent >= daySecondsFrom && daySecondsCurrent < daySecondsTo);
}

/**
 * Runs the default fix event sequence (only if applicable).
 */
void runDefaultFixEvent(uint32_t now)
{
    if (!isAlternativeFixEventApplicable()) {
        debugPrintln("Default fix event started.");
        getGpsFixAndTransmit();
    }
}

/**
 * Runs the alternative fix event sequence (only if it set and is within the set time).
 */
void runAlternativeFixEvent(uint32_t now)
{
    if (isAlternativeFixEventApplicable()) {
        debugPrintln("Alternative fix event started.");
        getGpsFixAndTransmit();
    }
}

/**
* Runs the "moving" fix event sequence (only if enabled, device is on the move and timeout hasn't elapsed).
*/
void runOnTheMoveFixEvent(uint32_t now)
{
    if (isOnTheMoveActivated) {
        if (now - lastOnTheMoveActivationTimestamp < params.getOnTheMoveTimeout() * 60) {
            debugPrintln("On-the-move fix event started.");
            getGpsFixAndTransmit();
        }
        else {
            // timeout elapsed -disable it completely until there is another on-the-move interrupt triggered
            debugPrintln("On-the-move has timed-out (no movement) -disabling.");
            isOnTheMoveActivated = false;
        }
    }
}

/**
 * Wakes up the lora module to put it back to sleep, i.e. extends the sleep period
*/
void runLoraModuleSleepExtendEvent(uint32_t now)
{
    debugPrintln("Extending LoRa module sleep period.");

    LoRa.extendSleep();
}

/**
 *  Checks validity of data, adds valid points to the points list, syncs the RTC
 */
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt)
{
    sodaq_wdt_reset();

    if (!isGpsInitialized) {
        debugPrintln("delegateNavPvt exiting because GPS is not initialized.");

        return;
    }

    // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
    ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
        NavPvt->year, NavPvt->month, NavPvt->day,
        NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
        NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

    // sync the RTC time
    if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
        uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

        // check if there is an actual offset before setting the RTC
        if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
            setNow(epoch);
        }
    }

    // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
    if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
        pendingReportDataRecord.setCourse((constrain(NavPvt->heading, 0, INT32_MAX) * 255) / (360 * 100000)); // scale and range to 0..255
        pendingReportDataRecord.setAltitude((int16_t)constrain(NavPvt->hMSL / 1000, INT16_MIN, INT16_MAX)); // mm to m
        pendingReportDataRecord.setLat(NavPvt->lat);
        pendingReportDataRecord.setLong(NavPvt->lon);
        pendingReportDataRecord.setSatelliteCount(NavPvt->numSV);
        pendingReportDataRecord.setSpeed((NavPvt->gSpeed * 36) / 10000); // mm/s converted to km/h

        isPendingReportDataRecordNew = true;
    }
}

/**
 * Tries to get a GPS fix and sends the data through LoRa if applicable.
 * Times-out after params.getGpsFixTimeout seconds.
 * Please see the documentation for more details on how this process works.
 */
bool getGpsFixAndTransmit()
{
    debugPrintln("Starting getGpsFixAndTransmit()...");

    if (!isGpsInitialized) {
        debugPrintln("GPS is not initialized, exiting...");

        return false;
    }

    bool isSuccessful = false;
    setGpsActive(true);

    pendingReportDataRecord.setSatelliteCount(0); // reset satellites to use them as a quality metric in the loop
    uint32_t startTime = getNow();
    while ((getNow() - startTime <= params.getGpsFixTimeout())
        && (pendingReportDataRecord.getSatelliteCount() < params.getGpsMinSatelliteCount()))
    {
        sodaq_wdt_reset();
        uint16_t bytes = ublox.available();

        if (bytes) {
            rtcEpochDelta = 0;
            isPendingReportDataRecordNew = false;
            ublox.GetPeriodic(bytes); // calls the delegate method for passing results

            startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

            // isPendingReportDataRecordNew guarantees at least a 3d fix or GNSS + dead reckoning combined
            // and is good enough to keep, but the while loop should keep trying until timeout or sat count larger than set
            if (isPendingReportDataRecordNew) {
                isSuccessful = true;
            }
        }
    }

    setGpsActive(false); // turn off gps as soon as it is not needed

    // populate all fields of the report record
    pendingReportDataRecord.setTimestamp(getNow());
    pendingReportDataRecord.setBatteryVoltage(getBatteryVoltage());
    pendingReportDataRecord.setBoardTemperature(getBoardTemperature());

    GpsFixDataRecord record;
    record.init();
    if (isSuccessful) {
        pendingReportDataRecord.setTimeToFix(pendingReportDataRecord.getTimestamp() - startTime);

        // add the new gpsFixDataRecord to the ringBuffer
        record.setLat(pendingReportDataRecord.getLat());
        record.setLong(pendingReportDataRecord.getLong());
        record.setTimestamp(pendingReportDataRecord.getTimestamp());

        gpsFixLiFoRingBuffer_push(&record);
    }
    else {
        pendingReportDataRecord.setTimeToFix(0xFF);

        // no need to check the buffer or the record for validity, default for Lat/Long is 0 anyway
        gpsFixLiFoRingBuffer_peek(0, &record);
        pendingReportDataRecord.setLat(record.getLat());
        pendingReportDataRecord.setLong(record.getLong());
    }

    if (params.getIsDebugOn()) {
        pendingReportDataRecord.printHeaderLn(&DEBUG_STREAM);
        pendingReportDataRecord.printRecordLn(&DEBUG_STREAM);
        debugPrintln();
    }

    transmit();

    return isSuccessful;
}

/**
 * Turns the GPS on or off.
 */
void setGpsActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        ublox.enable();
        ublox.flush();

        sodaq_wdt_safe_delay(80);

        PortConfigurationDDC pcd;

        uint8_t maxRetries = 6;
        int8_t retriesLeft;

        retriesLeft = maxRetries;
        while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.getPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.getPortConfigurationDDC(&pcd) failed!");

            return;
        }

        pcd.outProtoMask = 1; // Disable NMEA
        retriesLeft = maxRetries;
        while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.setPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.setPortConfigurationDDC(&pcd) failed!");

            return;
        }

        ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
        ublox.funcNavPvt = delegateNavPvt;
    }
    else {
        ublox.disable();
    }
}

/**
 * Turns the LoRa module on or off (wake up or sleep)
 */
void setLoraActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        LoRaBee.wakeUp();
    }
    else {
        LoRaBee.sleep();
    }
}

/**
 * Initializes the accelerometer or puts it in power-down mode
 * for the purpose of reading its temperature delta.
*/
void setAccelerometerTempSensorActive(bool on)
{
    // if on-the-move is initialized then the accelerometer is enabled anyway
    if (isOnTheMoveInitialized) {
        return;
    }

    if (on) {
        accelerometer.enableAccelerometer(LSM303AGR::LowPowerMode, LSM303AGR::HrNormalLowPower100Hz, LSM303AGR::XYZ, LSM303AGR::Scale2g, true);
        sodaq_wdt_safe_delay(30); // should be enough for initilization and 2 measurement periods
    }
    else {
        accelerometer.disableAccelerometer();
    }
}

/**
 * Prints the cause of the last reset to the given stream.
 *
 * It uses the PM->RCAUSE register to detect the cause of the last reset.
 */
static void printCpuResetCause(Stream& stream)
{
    stream.print("CPU reset by");

    if (PM->RCAUSE.bit.SYST) {
        stream.print(" Software");
    }

    // Syntax error due to #define WDT in CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/samd21j18a.h
    // if (PM->RCAUSE.bit.WDT) {
    if ((PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0) {
        stream.print(" Watchdog");
    }

    if (PM->RCAUSE.bit.EXT) {
        stream.print(" External");
    }

    if (PM->RCAUSE.bit.BOD33) {
        stream.print(" BOD33");
    }

    if (PM->RCAUSE.bit.BOD12) {
        stream.print(" BOD12");
    }

    if (PM->RCAUSE.bit.POR) {
        stream.print(" Power On Reset");
    }

    stream.print(" [");
    stream.print(PM->RCAUSE.reg);
    stream.println("]");
}

/**
 * Prints a boot-up message that includes project name, version,
 * and Cpu reset cause.
 */
static void printBootUpMessage(Stream& stream)
{
    stream.println("** " PROJECT_NAME " - " VERSION " **");

    getHWEUI();
    stream.print("LoRa HWEUI: ");
    for (uint8_t i = 0; i < sizeof(loraHWEui); i++) {
        stream.print((char)NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(loraHWEui[i])));
        stream.print((char)NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(loraHWEui[i])));
    }
    stream.println();

    stream.print(" -> ");
    printCpuResetCause(stream);

    stream.println();
}

/**
 * Callback from Config.reset(), used to override default values.
 */
void onConfigReset(void)
{
    setDevAddrOrEUItoHWEUI();

#ifdef DEFAULT_TEMPERATURE_SENSOR_OFFSET
    params._temperatureSensorOffset = DEFAULT_TEMPERATURE_SENSOR_OFFSET;
#endif

#ifdef DEFAULT_LORA_PORT
    params._loraPort = DEFAULT_LORA_PORT;
#endif

#ifdef DEFAULT_IS_OTAA_ENABLED
    params._isOtaaEnabled = DEFAULT_IS_OTAA_ENABLED;
#endif

#ifdef DEFAULT_DEVADDR_OR_DEVEUI
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_DEVADDR_OR_DEVEUI) > sizeof(params._devAddrOrEUI));

    strcpy(params._devAddrOrEUI, DEFAULT_DEVADDR_OR_DEVEUI);
#endif

#ifdef DEFAULT_APPSKEY_OR_APPEUI
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_APPSKEY_OR_APPEUI) > sizeof(params._appSKeyOrEUI));

    strcpy(params._appSKeyOrEUI, DEFAULT_APPSKEY_OR_APPEUI);
#endif

#ifdef DEFAULT_NWSKEY_OR_APPKEY
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_NWSKEY_OR_APPKEY) > sizeof(params._nwSKeyOrAppKey));

    strcpy(params._nwSKeyOrAppKey, DEFAULT_NWSKEY_OR_APPKEY);
#endif

#ifdef DEBUG
    params._isDebugOn = true;
#endif

}

void getHWEUI()
{
    // only read the HWEUI once
    if (!isLoraHWEuiInitialized) {
        if (initLora(LORA_INIT_SKIP_CONSOLE_MESSAGES, LORA_INIT_SKIP_JOIN)) {
            sodaq_wdt_safe_delay(10);
            uint8_t len = LoRa.getHWEUI(loraHWEui, sizeof(loraHWEui));

            if (len == sizeof(loraHWEui)) {
                isLoraHWEuiInitialized = true;
            }
        }
    }
}

void setDevAddrOrEUItoHWEUI()
{
    getHWEUI();

    if (isLoraHWEuiInitialized) {
        for (uint8_t i = 0; i < sizeof(loraHWEui); i++) {
            params._devAddrOrEUI[i * 2] = NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(loraHWEui[i]));
            params._devAddrOrEUI[i * 2 + 1] = NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(loraHWEui[i]));
        }
    }
}
