/**
 * Copyright 2016 Willem Eradus
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **/

#ifndef UBLOX_H
#define UBLOX_H

#include <Wire.h>

typedef struct __attribute__((packed,aligned(1))) NavigationPositionVelocityTimeSolution {
    uint32_t    iTOW;       // 00 GPS time of week of the navigation epoch.
    uint16_t    year;       // 04 Year UTC
    uint8_t     month;      // 06 Month, range 1..12 (UTC)
    uint8_t     day;        // 07 Day of month, range 1..31 (UTC)
    uint8_t     hour;       // 08 Hour of day, range 0..23 (UTC)
    uint8_t     minute;     // 09 Minute of hour, range 0..59 (UTC)
    uint8_t     seconds;    // 10 Seconds of minute, range 0..60 (UTC)
    uint8_t     valid;      // 11 Validity Flags (see graphic below)
    uint32_t    tAcc;       // 12 Time accuracy estimate (UTC)
    int32_t     nano;       // 16 Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t     fixType;    // 20 GNSSfix Type, range 0..5
    uint8_t     flags;      // 21 Fix Status Flags
    uint8_t     flags2;     // 22 Reserved
    uint8_t     numSV;      // 23 Number of satellites used in Nav Solution
    int32_t     lon;        // 24 Longitude
    int32_t     lat;        // 28 Latitude
    int32_t     height;     // 32 Height above Ellipsoid
    int32_t     hMSL;       // 36 Height above mean sea level
    uint32_t    hAcc;       // 40 Horizontal Accuracy Estimate
    uint32_t    vAcc;       // 44 Vertical Accuracy Estimate
    int32_t     velN;       // 48 NED north velocity
    int32_t     velE;       // 52 NED east velocity
    int32_t     velD;       // 56 NED down velocity
    int32_t     gSpeed;     // 60 Ground Speed (2-D)
    int32_t     heading;    // 64 Heading of motion 2-D
    uint32_t    sAcc;       // 68 Speed Accuracy Estimate
    uint32_t    headingAcc; // 72 Heading Accuracy Estimate
    uint16_t    pDOP;       // 76 Position DOP
    uint8_t   reserved1[6]; // 78 Reserved
    int32_t     headVeh;    // 84 Heading of vehicle (2-D)
    uint8_t   reserved2[4]; // 88 Reserved
} NavigationPositionVelocityTimeSolution;


typedef struct __attribute__((packed,aligned(1))) PortConfigurationDDC {
    uint8_t     portID;
    uint8_t     reserved0;
    uint16_t    txReady;
    uint32_t    mode;
    uint32_t    reserved3;
    uint16_t    inProtoMask;
    uint16_t    outProtoMask;
    uint16_t    flags;
    uint16_t    reserved5;
} PortConfigurationDDC;

typedef struct __attribute__((packed,aligned(1))) TimePulseParameters {
    uint8_t     tpIdx;
    uint8_t     reserved0;
    uint16_t    reserved1;
    int16_t     antCableDelay;
    int16_t     rfGroupDelay;
    uint32_t    freqPeriod;
    uint32_t    freqPeriodLock;
    uint32_t    pulseLenRatio;
    uint32_t    pulseLenRatioLock;
    int32_t     userConfigDelay;
    uint32_t    flags;

} TimePulseParameters;

typedef struct __attribute__((packed,aligned(1))) TimePulseTimedata {
    uint32_t    towMS;          // ms Time pulse time of week according to time base
    uint32_t    towSubMS;       // ms Submillisecond part of TOWMS
    int32_t     qErr;           // ps Quantization error of time pulse.
    uint16_t    week;           // weeks Time pulse week number according to time base
    uint8_t     flags;
    uint8_t     reserved1;
} TimePulseTimedata;

typedef struct __attribute__((packed,aligned(1))) NavigationEngineSettings {
    uint16_t mask;              // Parameters Bitmask. Only the masked parameters will be applied.
    uint8_t  dynModel;          // Dynamic platform model:
                                // 0: portable
                                // 2: stationary
                                // 3: pedestrian
                                // 4: automotive
                                // 5: sea
                                // 6: airborne with <1g acceleration
                                // 7: airborne with <2g acceleration
                                // 8: airborne with <4g acceleration
                                // 9: wrist worn watch (not supported in protocol versions less than 18)
    uint8_t  fixMode;           // Position Fixing Mode:
                                // 1: 2D only 
                                // 2: 3D only 
                                // 3: auto 2D/3D
    int32_t  fixedAlt;          // Fixed altitude in meters (mean sea level) for 2D fix mode
    uint32_t fixedAltVar;       // Fixed altitude variance for 2D mode in m^2.
    int8_t   minElev;           // Minimum Elevation in degrees for a GNSS satellite to be used in NAV
    uint8_t  drLimit;           // Reserved
    uint16_t pDop;              // Position DOP Mask to use
    uint16_t tDop;              // Time DOP Mask to use
    uint16_t pAcc;              // Position Accuracy Mask in meters
    uint16_t tAcc;              // Time Accuracy Mask in meters
    uint8_t  staticHoldThresh;  // Static hold threshold in cm/s
    uint8_t  dgnssTimeout;      // DGNSS timeout in seconds
    uint8_t  cnoThreshNumSats;  // Number of sattelites required to have C/NO above cnoThresh 
                                // for a fix to be attempted
    uint8_t  cnoThresh;         // C/N0 threshold for deciding whether to attempt a fix in dBHz
    uint8_t  reserved1[2];      // 2 bytes reserved
    uint16_t staticHoldMaxDist; // Static hold distance threshold before quitting static hold
    uint8_t  utcStandard;       // UTC standard to be used:
                                // 0: Automatic; receiver selects based on GNSS configuration 
                                //    (see GNSS time bases).
                                // 3: UTC as operated by the U.S. Naval Observatory (USNO); 
                                //    derived from GPS time
                                // 6: UTC as operated by the former Soviet Union; derived from 
                                //    GLONASS time
                                // 7: UTC as operated by the National Time Service Center,
                                //    China; derived from BeiDou time
                                //    (not supported in protocol versions less than 16).
    uint8_t  reserved2[5];      // 5 bytes reserved
} NavigationEngineSettings;

enum Messages {
    UBX_NAV_PVT = 0x0107,
    UBX_TIM_TP  = 0x0d01,
    NMEA_CGA    = 0xf000,
    NMEA_GLL,
    NMEA_GSA,
    NMEA_GSV,
    NMEA_RMC,
    NMEA_VTG,
    NMEA_GRS,
    NMEA_GST,
    NMEA_ZDA,
    NMEA_GBS,
    NMEA_DTM,
    NMEA_GNS    = 0xf00d,
    NMEA_GPQ    = 0xf040,
    NMEA_TXT,
    NMEA_GNQ,
    NMEA_GLQ
};

class UBlox {

public:
    // NavigationPositionVelocityTimeSolution *NavPvt;
    // TimePulseParameters *CfgTp;

    UBlox   ();
    UBlox   (TwoWire& Wire,uint8_t address);
    void    CfgMsg(uint16_t Msg,uint8_t rate);
    int     available();

    bool    getPortConfigurationDDC(PortConfigurationDDC* pcd);
    int     setPortConfigurationDDC(PortConfigurationDDC *pcd);

    bool    getTimePulseParameters(uint8_t tpIdx,TimePulseParameters* tpp);
    int     setTimePulseParameters(TimePulseParameters *Tpp);
    
    bool    getNavEngineSettings(NavigationEngineSettings* navDynModel);
    bool    setNavEngineSettings(NavigationEngineSettings* navDynModel);

    void    GetPeriodic();
    void    GetPeriodic(int bytes);
    void    enable ();
    void    disable ();
    void    flush();
    void    reset();
    bool    exists() const;

    // function pointers
    void    (*funcNavPvt) (NavigationPositionVelocityTimeSolution*) = NULL;

    // Debug helper
    void    db_printf(const char *message,...);

    int     process(uint8_t);
    void    sendraw();

private:
    int     send(uint8_t *buffer,int n);
    int     wait();
    bool    wait(uint16_t rid,int reqLength,void *d);
    void 	dispatchMessage(int id);
    //
    TwoWire&    Wire_;
    uint8_t     address_;
    uint8_t     *p_;
    int         state_ = 0;
    uint16_t    AckedId_;
    uint16_t    plLength_;
    uint16_t    Id_;

    struct payload{
        uint16_t length;
        uint8_t buffer[256];
    } payLoad_;
    
    
};

#endif // UBLOX_H
