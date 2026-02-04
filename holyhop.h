// MIT License
//
// Copyright (c) 2026 John Andrew McInnes
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// C++ application radio protocol for radar and other sensors. You could describe it as
//  "A sleepy tree network". The design priority is saving batteries and allowing for meshing/relaying.
//  C++ type checking and templates are part of the philosophy here. Runtime checks where needed.
//  Fixed size packets, strings, buffers, etc.
//  #pragma packed for minimum size on the wire, little endian CPUs are assumed on both ends.
//
// (C) 2024 John McInnes
//
// Some Terminology:
//  tlm - telemetry
//  TX - transmit a packet
//  RX - receieve a packet
//  sensor a.k.a. node,sensornode,relay,instrument - basically something that is not a base that talks, can
//    relay packets, and goes to sleep most of the time.
//  the Base interval a.k.a.  network interval, RX Window, wake interval
//    - when sensors wake up and communicate with the Base.
//
// 2025/02/20 first version glommed together from bible camp sensor project.
// 2025/04/14 adding relay capability
// 2025/05/07 fixes and enhancements
// 2025+ lots of fixes and enhancements
//
// todo: FCC airtime 400ms duty-cycle apply to Lora P2P and channel hopping?
// todo: improve relay branch failure propagation
// todo: intelligently rate limit comms failure retries
// todo: dynamic-scalable RX window - unlimited nodes! well as many as you care to try.
// todo: encryption, AES is probably natively supported by the nRF
//

// todo brief description of 'mesh' protocol:
//  Periodic telemetry packets double as a node's network state announcement (see base_hops field).
//  The base and any neighboring nodes use this to learn about other nodes.
//  To be 'connected' is to know what neighbor to relay packets thru to reach the base (could be direct to base).
//  This is the node's 'uplink'. So uplink information propagates outward from the base - as does disconnection/failure.
//  Uplink disconnection/failure is when a node hasn't heard from base in X amount of time, or if an uplink
//  announces that it has lost it's *own* uplink. A node will then decide that is is disconnected and try a different
//  route to base if knows one or hears one become available (again). In this way packets make it to the base (root).
//  To reply, that last successful route is reversed.
//
//  This is all complicated by the fact that the whole network is only awake for a short window of time during which
//  telemetry packets are sent in a fairly strict order. more explanation to come


#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <array>
#ifdef ARDUINO
#include "seriallog.h"
typedef String HHStrClass;  // avoid std::string on embedded platforms
#else
#include <deque>
#include <vector>
#include <string>
typedef std::string HHStrClass;
#include "teestream.h"
#endif

// Arduino project wide defines are difficult so set the defs here
#define HOLYHOP_BASE      // code for a BASE
//#define HOLYHOP_SENSOR    // code for a SENSOR

#if !defined(HOLYHOP_BASE) && !defined(HOLYHOP_SENSOR)
#error "Compilation failed: either HOLYHOP_BASE or HOLYHOP_SENSOR must be defined"
#endif
#if defined(HOLYHOP_BASE) && defined(HOLYHOP_SENSOR)
#error "Both HOLYHOP_BASE and HOLYHOP_SENSOR are defined; please choose only one."
#endif

// time constants are milliseconds unless labeledd _SEC
#define HOLYHOP_VERSION 0x65        // single byte protocol version
#define HOLYHOP_NAME_SIZE 13        // includes term. null
#define HOLYHOP_DID_SIZE 4          // device id/address
#define HOLYHOP_DID_HEXSTR_SIZE (HOLYHOP_DID_SIZE * 2 + 1)
#define HOLYHOP_MAX_NODES 1000
#define HOLYHOP_MAX_CMDSQ 6
#define HOLYHOP_MAX_HOP_COUNT 10     // max # of hops allowed for a packet.. affects timings
#define HOLYHOP_FORGET_NODE_SEC (60*60*6) // if we dont hear any traffic from a node after ahile forget about it
#define HOLYHOP_PING_NODES_SEC (HOLYHOP_FORGET_NODE_SEC - 60*60) // how often base 'pings' nodes so they don't forget it
#define HOLYHOP_RX_WINDOW_SEC 30     // how long nodes are awake for TX-RX commands and telemetry
#define HOLYHOP_TX_EXTRA_WAIT_SEC 10 // Base wait to talk to waking nodes because they will be doing telemetry TX
                                     // todo: calculate this by tallying node tlm_offset times
#define HOLYHOP_PRE_RX_WINDOW_SEC 10 // if this close to start of RX window don't send new commands
//#define HOLYHOP_TX_TIMEOUT 10000
#define HOLYHOP_REPLY_TIMEOUT 7000  // milliseconds to listen for command reply. give some time but dont eat the whole RX window
                                    // todo: calculate this by using node's tlm_offset time
#define HOLYHOP_TLM_PKT_TIME 576    // time on air in ms for tlm packet (todo: runtime calc this based on LoRa config)
//#define HOLYHOP_TX_RELAY_TIME 10    // milliseconds it takes a sensor node to turn around and relay a pkt
                                    // ..(not counting LoRa turnaround below)
#define HOLYHOP_DEFAULT_TLM_INTERVAL_SEC (5*60) // a default interval until we hear from Base
#define HOLYHOP_CMD_EXPIRE_SEC (12*60*60)   // 12 hours
#define HOLYHOP_CMD_MAX_RETRY 2             // TX retries
#define HOLYHOP_CMD_MAX_QCMDS 15            // max size of server cmd_queue and hist_queue
#define LORA_MAX_PKT_SIZE 256
#define LORA_TURNAROUND_DELAYMS 50  // Time to give a node to process a packet and switch radio modes.
                                    // Some radios need alot of time to go from TX to RX or RX to TX..
                                    // the base rpi for one. You can switch modes quickly, but the radio might
                                    // not actually be able to RX a weak signal for awhile.
                                    // Going from RX->TX we pause (to give other nodes time to go from TX->RX)
#define LORA_CAD_DELAYMS 130        // Time in ms for 1 Channel Activity Detection attempt to complete

static_assert(HOLYHOP_PING_NODES_SEC > 60);
static_assert(HOLYHOP_TX_EXTRA_WAIT_SEC < HOLYHOP_RX_WINDOW_SEC);
// todo: assert that maxnodes * rxwindow * maxhops all checks out

enum KnownOUI_t : uint8_t {     // see IsHHPktValid..
    OUI_ZERO = 0,               // reserved
    BROADCAST = 1,              // special: dest is any/all local nodes (direct hop)
    BASE = 2,                   // special: dest is any/all Bases, src is Base with unique id {BASE,X,X,X}
    OUI_RAK_NRF52 = 10,         // a RAK Wireless NRF52 device
};

// todo: compile-time hardcode/mate cmd with its reply
enum PacketType_t : uint8_t {
    UNSET = 0,
    // Command from base to sensor. Commands have replies.
    CMD_RESET = 1,
    CMD_GET_VERSION,
    CMD_GET_NAME,
    CMD_SET_NAME,
    CMD_GET_INTERVAL,
    CMD_SET_INTERVAL,
    CMD_CONFIG_LORA,
    CMD_CONFIG_RADAR,
    CMD_GET_RADAR_CONFIG,
    CMD_BEGIN_DFU_UPLOAD,
    CMD_DFU_UPLOAD,
    CMD_VERIFY_DFU_UPLOAD,
    CMD_BLINK_LED,
    CMD_GET_PREFERRED_UPLINK,
    CMD_SET_PREFERRED_UPLINK,

    // reply to a command from sensors to base see IsHHReplyPkt()
    REPLY_VERSION = 60,
    REPLY_SUCCESS,
    REPLY_FAILURE,
    REPLY_NAME,
    REPLY_INTERVAL,
    REPLY_RADAR_CONFIG,
    REPLY_DFU_UPLOAD,
    REPLY_VERIFY_DFU_UPLOAD,
    REPLY_PREFERRED_UPLINK,

    // special/other packet types
    //HOP_INFO = 85,
    RADAR_TELEMETRY = 100,
    RADAR_ALLM_TELEMETRY,
    RADAR_GNSS_TELEMETRY
};

enum PacketReplyStatus_t : uint8_t{
    RSTATUS_UNSET = 0,
    RSTATUS_SUCCESS = 1,
    RSTATUS_FAILURE,
    RSTATUS_TIMEOUT   // no response, out of retries
};

///////////////////////////////////
// Packed structs for transmission
#pragma pack(push, 1)
// PacketDeviceID_t is for packing bytes. Better to use HHDeviceAddress_t if you need to work with device ids
struct PacketDeviceID_t {
    uint8_t TheID[HOLYHOP_DID_SIZE];

    bool IsMatch(PacketDeviceID_t other_id) const{
        return memcmp(TheID, other_id.TheID, sizeof(TheID)) == 0;
        //return strncmp( TheID, pstr, sizeof(TheID) ) == 0;
    }

    bool IsBroadcast() const{
        return TheID[0] == KnownOUI_t::BROADCAST;
    }

    bool IsBase() const{
        return TheID[0] == KnownOUI_t::BASE;
    }

    bool IsKnownOUI(){
        switch(TheID[0]){
        case BROADCAST:
        case BASE:
        case OUI_RAK_NRF52:
            return true;
        default:
            return false;
        }
    }

    bool operator ==(const PacketDeviceID_t &other) const{
        return IsMatch(other);
    }
/*
    uint32_t ToDWORD() const{
        uint32_t dword = (TheID[0] << 24) | (TheID[1] << 16) | (TheID[2] << 8) | TheID[3];
        return dword;
    }

    static PacketDeviceID_t FromDWORD( uint32_t dwid ){
        PacketDeviceID_t the_id;
        the_id.SetID( dwid );
        return the_id;
    }
*/
    static PacketDeviceID_t GenericBaseID(){
        PacketDeviceID_t did;
        did.TheID[0] = KnownOUI_t::BASE;
        did.TheID[1] = 0;
        did.TheID[2] = 0;
        did.TheID[3] = 0;
        return did;
    }
};

// string encapsulator. StrArraySize must include terminating null byte
template<int StrArraySize>
struct PacketString_t {

    void MakeSafe(){
        StrA[StrArraySize-1] = 0; // terminate string
    }
    size_t Len() {
        return strnlen(StrA, MaxLen());
    }
    size_t MaxLen() {
        return StrArraySize - 1;
    }
    const char *Get() {
        MakeSafe();
        return StrA;
    }
    void Set(const char *src, size_t s_len) {
        memset(StrA, 0, sizeof(StrA));  // zero it first
        memcpy(StrA, src, std::min(s_len, MaxLen()));
    }
private:
    char StrA[StrArraySize];  // string array
};

struct PacketHeader_t {
    PacketHeader_t() = delete;
    explicit PacketHeader_t( PacketType_t pkt_type ) : PktType(pkt_type){}

    PacketDeviceID_t RelayTo;   // node that should receive this packet
    PacketDeviceID_t RelayBy;   // node that transmitted/relayed the packet
    PacketDeviceID_t Dest;      // final destination of packet
    PacketDeviceID_t Src;       // origin of packet
    PacketType_t PktType;
    uint8_t HopCount;           // how many hops packet has taken
    const uint8_t HHVer = HOLYHOP_VERSION;
};

// Command packets and Reply packets have a ref_id for matching with each other.
// Create a new command reference id. Just a random number. Not meant for security, more for
// ..sanity checking and debugging comms protocol.
static uint8_t NewPacketRefID(){
    return rand() % 256;
}

// all command packets
template<PacketType_t PktTypeCode, class TPayloadType_t>
struct PacketCommand_t{
    PacketCommand_t() : Header(PktTypeCode), RefID( NewPacketRefID() ){}

    PacketHeader_t Header;
    uint8_t RefID;
    TPayloadType_t Payload;
};

// all reply packets
template<PacketType_t PktTypeCode, class TPayloadType_t>
struct PacketReply_t{
    PacketReply_t() = delete;
    PacketReply_t( uint8_t aref_id ) : Header(PktTypeCode), RefID(aref_id){}

    PacketHeader_t Header;
    uint8_t RefID;
    TPayloadType_t Payload;
};

// all 'telemetry' packets
template<PacketType_t PktTypeCode, class TPayloadType_t>
struct PacketTelemetry_t{
    PacketTelemetry_t() = delete;
    PacketTelemetry_t( uint8_t base_hops ) : Header(PktTypeCode), BaseHops(base_hops){}

    PacketHeader_t Header;
    uint8_t BaseHops;           // sensor's hop count from base. 0 if no route to base.
    TPayloadType_t Payload;
};

//
// Packet Payload definitions
//
struct PayloadEmpty_t{
    // empty!
};

struct PayloadGetVersion_t {
    const uint8_t hh_version = HOLYHOP_VERSION;  // send our hh version with the command
};

struct PayloadSetName_t {
    PacketString_t<HOLYHOP_NAME_SIZE> name;
};

struct PayloadSetPreferredUplink_t{
    PacketDeviceID_t uplink;
};

struct PayloadSetInterval_t {
    int32_t measurement_ms;     // how often we take sensor measurements
    int32_t tlm_int_sec;        // telemetry interval. how often we wake to TX and RX w/ Base.
    int32_t next_tlm_ms;        // time until next interval wake event. for getting in sync.
    uint16_t tlm_offset_ms;     // offset in ms from interval start that sensor should start to TX telemetry
                                //  to avoid TX collisions.
    int64_t etime_sec;          // current time in epoch seconds
};

struct PayloadConfigLoRa_t {
    uint32_t frequency_hz;
    int16_t tx_power_dbm;
    uint8_t bandwidth_code;
    uint8_t spreading_factor;
    uint8_t codingrate_code;
};

struct PayloadConfigRadar_t {
    int32_t start_dist_mm;
    int32_t end_dist_mm;
    int8_t max_profile;
    int8_t peak_sorting;
    int8_t threshold_method;
    int32_t threshold_sensitivity;
    int32_t fixed_amp_threshold;
    int32_t fixed_str_threshold;
    int32_t signal_quality;
    uint16_t max_step_count;
    int8_t reflector_shape;
    uint8_t close_range_leakage;
};

struct PayloadBeginDFUUpload_t {
    uint32_t dfu_size;
    uint16_t dfu_crc;
};

struct PayloadDFUUpload_t {
    uint8_t num_bytes;
    uint32_t dfu_offset;
    uint8_t barray[210];
};

struct PayloadReplyVersion_t {
    const uint8_t hh_version = HOLYHOP_VERSION;
    uint16_t fw_version;    // sensor firmware ver
    uint32_t bl_version;    // bootloader ver
};

struct PayloadReplyName_t {
    PacketString_t<HOLYHOP_NAME_SIZE> name;
};

struct PayloadReplyPreferredUplink_t{
    uint32_t uplink;
};

struct PayloadReplyInterval_t {
    int32_t measurement_ms;     // how often we take sensor measurements
    int32_t tlm_int_sec;        // how often we TX telemetry or RX
};

struct PayloadReplyRadarConfig_t {
    int32_t start_dist_mm;
    int32_t end_dist_mm;
    int8_t max_profile;
    int8_t peak_sorting;
    int8_t threshold_method;
    int32_t threshold_sensitivity;
    int32_t fixed_amp_threshold;
    int32_t fixed_str_threshold;
    int32_t signal_quality;
    uint16_t max_step_count;
    int8_t reflector_shape;
    uint8_t close_range_leakage;
};

struct PayloadReplyDFUUpload_t {
    bool fsuccess;              // true if dfu operation successful
    uint32_t current_offset;    // how many bytes sensor has received so far, from beg.
};

struct PayloadReplyVerifyDFUUpload_t {
    bool fsuccess;              // true if dfu operation successful
};

struct PayloadRadarTelemetry_t { // standard radar measurement
    uint16_t wake_count;
    int16_t batt_voltage;
    int16_t last_rssi;
    int8_t last_snr;
    uint16_t radar_dist0;
    uint16_t radar_dist1;
    uint16_t radar_dist2;
    int16_t radar_strength0;
    uint16_t radarwide_dist0;
    uint16_t radarwide_dist1;
    uint16_t radarwide_dist2;
    int16_t radarwide_strength0;
    int8_t temp_c;
    uint32_t uplink;            // our current uplink address as DWORD
};

struct PayloadRadarAllMTelemetry_t { // all radar peak measurements
    uint16_t wake_count;
    int16_t batt_voltage;
    int16_t last_rssi;
    int8_t last_snr;
    uint16_t radar_dist0;
    uint16_t radar_dist1;
    uint16_t radar_dist2;
    uint16_t radar_dist3;
    uint16_t radar_dist4;
    uint16_t radar_dist5;
    uint16_t radar_dist6;
    uint16_t radar_dist7;
    uint16_t radar_dist8;
    uint16_t radar_dist9;
    int16_t radar_strength0;
    int16_t radar_strength1;
    int16_t radar_strength2;
    int16_t radar_strength3;
    int16_t radar_strength4;
    int16_t radar_strength5;
    int16_t radar_strength6;
    int16_t radar_strength7;
    int16_t radar_strength8;
    int16_t radar_strength9;
    int8_t temp_c;
    uint32_t uplink;            // our current uplink address as DWORD
};

struct PayloadRadarGNSSTelemetry_t { // standard radar measurement w/ GNSS
    uint16_t wake_count;
    int16_t batt_voltage;
    int16_t last_rssi;
    int8_t last_snr;
    uint16_t radar_dist0;
    uint16_t radar_dist1;
    uint16_t radar_dist2;
    uint16_t radar_dist3;
    uint16_t radar_dist4;
    uint16_t radar_dist5;
    uint16_t radar_dist6;
    uint16_t radar_dist7;
    uint16_t radar_dist8;
    uint16_t radar_dist9;
    int16_t radar_strength0;
    int16_t radar_strength1;
    int16_t radar_strength2;
    int16_t radar_strength3;
    int16_t radar_strength4;
    int16_t radar_strength5;
    int16_t radar_strength6;
    int16_t radar_strength7;
    int16_t radar_strength8;
    int16_t radar_strength9;
    int8_t temp_c;
    uint32_t uplink;            // our current uplink address as DWORD
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint8_t fix_type;           // GNSS fix type
};

//
// Command packets
//
typedef struct PacketCommand_t<CMD_RESET, PayloadEmpty_t> PacketCommandReset_t;
typedef struct PacketCommand_t<CMD_GET_VERSION, PayloadGetVersion_t> PacketCommandGetVersion_t;
typedef struct PacketCommand_t<CMD_GET_NAME, PayloadEmpty_t> PacketCommandGetName_t;
typedef struct PacketCommand_t<CMD_SET_NAME, PayloadSetName_t> PacketCommandSetName_t;
typedef struct PacketCommand_t<CMD_GET_PREFERRED_UPLINK, PayloadEmpty_t> PacketCommandGetPreferredUplink_t;
typedef struct PacketCommand_t<CMD_SET_PREFERRED_UPLINK, PayloadSetPreferredUplink_t> PacketCommandSetPreferredUplink_t;
// The telemetry interval is: how often the sensors wake up to send telemetry. They also listen for incoming
//   packets for a while (the RX window of time). Then then the sensors go back to sleep. (as far as the
//   base knows). At time of interval start, the sensors can be expected to start sending their latest
//   telemetry to the Base, in some kind of order, 1 sensor at a time. Also the RX window begins. During the
//   RX window a sensor can be expected to RX commands, TX replies, and to relay packets. During the beginning
//   of the RX window, the Base can be expected to be listening for incoming tlm. After sufficient time
//   has passed for that, the Base will use the rest of the RX window to send commands to sensor nodes.
//
// So the RX window opens:
//      - base is listening, sensors are sending tlm.
//      - base tlm wait ends: all (known) sensors have sent tlm
//            - base now sends pending cmds to sensors.
//            - base receives replies to cmds.
//            - any new sensors or late tlm is sent to base. base is listening.
//      - RX window closes. Sensors go to sleep and cannot relay, except..
//      - If a sensor is being talked to it will stay awake.
//      - base will continue any long running cmds (dfu upload)
//      - base is always listening for new nodes and traffic. it does not sleep.
//
typedef struct PacketCommand_t<CMD_GET_INTERVAL, PayloadEmpty_t> PacketCommandGetInterval_t;
typedef struct PacketCommand_t<CMD_SET_INTERVAL, PayloadSetInterval_t> PacketCommandSetInterval_t;

typedef struct PacketCommand_t<CMD_GET_RADAR_CONFIG, PayloadEmpty_t> PacketCommandGetRadarConfig_t;
typedef struct PacketCommand_t<CMD_CONFIG_LORA, PayloadConfigLoRa_t> PacketCommandConfigLoRa_t;
typedef struct PacketCommand_t<CMD_CONFIG_RADAR, PayloadConfigRadar_t> PacketCommandConfigRadar_t;
// prepare to receive dfu
typedef struct PacketCommand_t<CMD_BEGIN_DFU_UPLOAD, PayloadBeginDFUUpload_t> PacketCommandBeginDFUUpload_t;
// upload a dfu chunk
typedef struct PacketCommand_t<CMD_DFU_UPLOAD, PayloadDFUUpload_t> PacketCommandDFUUpload_t;
// finish dfu & verify
typedef struct PacketCommand_t<CMD_VERIFY_DFU_UPLOAD, PayloadEmpty_t> PacketCommandVerifyDFUUpload_t;
// blink the sensor's LEDs in a very visible way
typedef struct PacketCommand_t<CMD_BLINK_LED, PayloadEmpty_t> PacketCommandBlinkLED_t;


//
// Reply packets
//
// generic success reply for commands without a specific reply
typedef struct PacketReply_t<REPLY_SUCCESS, PayloadEmpty_t> PacketReplySuccess_t;

// generic reply last command failed
typedef struct PacketReply_t<REPLY_FAILURE, PayloadEmpty_t> PacketReplyFailure_t;

typedef struct PacketReply_t<REPLY_VERSION, PayloadReplyVersion_t> PacketReplyVersion_t;
typedef struct PacketReply_t<REPLY_NAME, PayloadReplyName_t> PacketReplyName_t;
typedef struct PacketReply_t<REPLY_PREFERRED_UPLINK, PayloadReplyPreferredUplink_t> PacketReplyPreferredUplink_t;
typedef struct PacketReply_t<REPLY_INTERVAL, PayloadReplyInterval_t> PacketReplyInterval_t;
typedef struct PacketReply_t<REPLY_RADAR_CONFIG, PayloadReplyRadarConfig_t> PacketReplyRadarConfig_t;

// DFU is in progress
typedef struct PacketReply_t<REPLY_DFU_UPLOAD, PayloadReplyDFUUpload_t> PacketReplyDFUUpload_t;

typedef struct PacketReply_t<REPLY_VERIFY_DFU_UPLOAD, PayloadReplyVerifyDFUUpload_t> PacketReplyVerifyDFUUpload_t;


//
// Telemetry packets are not Reply packets or Command packets. They are unsolicited/special.
//
typedef struct PacketTelemetry_t<RADAR_TELEMETRY, PayloadRadarTelemetry_t> PacketRadarTelemetry_t;
typedef struct PacketTelemetry_t<RADAR_ALLM_TELEMETRY, PayloadRadarAllMTelemetry_t> PacketRadarAllMTelemetry_t;
typedef struct PacketTelemetry_t<RADAR_GNSS_TELEMETRY, PayloadRadarGNSSTelemetry_t> PacketRadarGNSSTelemetry_t;

/*
struct PacketHopInfo_t {     // broadcast this to inform other nodes
    PacketHeader_t<HOP_INFO> Header;
    struct PayloadHopInfo_t {
        int16_t base_hops;          // how many hops we are from base (0-unknown,1-direct)
        int32_t telemetry_sec;      // telemetry interval (a.k.a. wake interval)
    } Payload;
}; */
#pragma pack(pop)

// a helper class for working with device id's (addresses)
class HHDeviceAddress_t{
public:
    HHDeviceAddress_t(){
        SetDW(0x00000000);
    }

    HHDeviceAddress_t(const PacketDeviceID_t &did){
        SetDID(did);
    }

    HHDeviceAddress_t(uint32_t dwid){
        SetDW(dwid);
    }

    HHDeviceAddress_t(uint8_t oui_byte, uint8_t ic0, uint8_t ic1, uint8_t ic2){
        IDArray[0] = oui_byte;
        IDArray[1] = ic0;
        IDArray[2] = ic1;
        IDArray[3] = ic2;
    }

    // @brief Set id from PacketDeviceID_t
    void SetDID( const PacketDeviceID_t &did){
        for( size_t i=0; i < sizeof(did.TheID); i++ ){
            IDArray[i] = did.TheID[i];
        }
    }

    // @brief Set id from chars
    void SetID(uint8_t oui_byte, uint8_t ic0, uint8_t ic1, uint8_t ic2) {
        IDArray[0] = oui_byte;
        IDArray[1] = ic0;
        IDArray[2] = ic1;
        IDArray[3] = ic2;
    }

    // @brief Set id from DWORD
    void SetDW( uint32_t dwid ){
        IDArray[0] = (dwid & 0xFF000000) >> 24;
        IDArray[1] = (dwid & 0x00FF0000) >> 16;
        IDArray[2] = (dwid & 0x0000FF00) >> 8;
        IDArray[3] = (dwid & 0x000000FF);
    }

    uint32_t ToDW(){
        uint32_t dwid = 0;
        dwid |= IDArray[0] << 24;
        dwid |= IDArray[1] << 16;
        dwid |= IDArray[2] << 8;
        dwid |= IDArray[3];
        return dwid;
    }

    void ToDID( PacketDeviceID_t &did ) const{
        for( size_t i=0; i < sizeof(did.TheID); i++ ){
            did.TheID[i] = IDArray[i];
        }
    }

    PacketDeviceID_t ToDID() const{
        PacketDeviceID_t did;
        ToDID(did);
        return did;
    }

    // @brief Write as hexadecimal ascii. doing it kind of ugly to be efficent on a microcontroller.
    // ..we dont want to create a bunch of C++ strings and other temporary objects.
    void ToHexStr(char stra[], size_t s_size) const {
        s_size = std::min(s_size, (size_t)HOLYHOP_DID_HEXSTR_SIZE); // safety check
        for (size_t i=0; i < HOLYHOP_DID_SIZE; i++) {
            snprintf(stra + i * 2, HOLYHOP_DID_HEXSTR_SIZE - i * 2, "%02X", IDArray[i]);
        }
    }

    bool IsMatch(const HHDeviceAddress_t other_id) const{
        return IDArray == other_id.IDArray;
    }

    bool IsMatch(const PacketDeviceID_t other_id) const{
        return ToDID() == other_id;
    }

    // @brief Set id from DWORD
    static HHDeviceAddress_t FromDWORD(uint32_t dwid){
        HHDeviceAddress_t a_id(dwid);
        return a_id;
    }

    void SetNull(){
        SetDW( 0x0 );
    }

    bool IsNull() const{
        return IsMatch( HHDeviceAddress_t() );
    }

    bool IsBase() const{
        return IDArray[0] == KnownOUI_t::BASE;
    }

    const char * c_str() const{
        ToHexStr(AString,sizeof(AString));
        return AString;
    }

    bool operator ==(const HHDeviceAddress_t &other) const{
        return IsMatch(other);
    }

    bool operator !=(const HHDeviceAddress_t &other) const{
        return !IsMatch(other);
    }

    HHDeviceAddress_t & operator =(const HHDeviceAddress_t &other){
        IDArray = other.IDArray;
        c_str();
        return *this;
    }

#ifndef ARDUINO
    operator const std::string() const{
        ToHexStr(AString,sizeof(AString));
        return std::string(AString);
    }
#endif
    operator const char *() const{
        return c_str();
    }

    std::array<uint8_t,HOLYHOP_DID_SIZE> IDArray;
private:
    mutable char AString[HOLYHOP_DID_HEXSTR_SIZE]; // helper for string conversion
};

#ifndef ARDUINO
// fmt:: helper for HHDeviceAddress_t
template <> struct fmt::formatter<HHDeviceAddress_t>: formatter<string_view> {
    // parse is inherited from formatter<string_view>.
    auto format(HHDeviceAddress_t dev_id, format_context& ctx) const -> format_context::iterator{
    string_view name = (const char *)dev_id;
        return formatter<string_view>::format(name, ctx);
    }
};
#endif

#ifdef HOLYHOP_BASE
struct HHQueuedCommand_t{
    bool fsend_now;             // if true packet should be sent immediately, not at wake time. for debugging.
    HHDeviceAddress_t dest_id;  // redundant?
    PacketType_t pkt_type;
    uint8_t ref_id;
    int16_t pkt_size;
    std::filesystem::path fw_path;  // if doing dfu, this is the fw bin file to use
    std::array<uint8_t,LORA_MAX_PKT_SIZE> cmd_pkt_buf = {};
};
#endif

struct HHQueuedPacket_t{
    PacketType_t pkt_type;
    uint16_t pkt_size;
    std::array<uint8_t,LORA_MAX_PKT_SIZE> pkt_array; // hey arduino doesn't like this: = {};
};

struct HHNodeInfo_t {
    HHDeviceAddress_t dev_id;   // device mesh id/address of pkt origin
    HHDeviceAddress_t rev_relay_id;// node's traffic was relayed to US by this node (could be itself)
    bool flocal;                // true if node is local to us (1 hop)
    uint16_t protocol_ver;      // HOLYHOP_VERSION
    uint16_t firmware_ver;      // sensor firmware version
    uint32_t bootloader_ver;    // sensor bootloader version
    HHStrClass name;            // device's friendly name
    time_t last_tx_time;        // unix time in sec when last we heard from node
    uint32_t tlm_offset_ms;     // offset from wake time that sensor should send it's telemetry
    uint16_t base_hops;         // how many hops from Base node reports that it is
    uint16_t relay_hops;        // how many hops last pkt from dev_id had taken thus far
    int16_t last_rssi;          // rssi of last received packet
    int8_t last_snr;            // SNR of last received packet
#ifdef HOLYHOP_BASE
    int32_t measurement_ms;     // how often sensor takes measurements
    PayloadReplyRadarConfig_t radar_cfg; // last received radar cfg if any
    uint32_t wake_count;        // last wake_count
    std::deque<HHQueuedCommand_t> cmd_queue;    // cmds to be sent
    std::deque<HHQueuedCommand_t> cmd_hist;     // sent commands. back() is current command
    int reply_wait_count;                       // count cmd retries. 0: no active command
    PacketReplyStatus_t reply_status;
    std::filesystem::path fw_path;              // firmware update file pathname, if set then node is doing DFU
    uint32_t fw_offset;                         // current offset of upload
    HHDeviceAddress_t preferred_uplink_id;      // node's preferred uplink if any
#endif
};
// typedef std::unordered_map<PacketDeviceID_t,HHNodeInfo_t> HHNodeInfoMap_t;
//
// // specializtion to make PacketDeviceID_t hashable
// template<>
// struct std::hash<PacketDeviceID_t>
// {
//     std::size_t operator()(const PacketDeviceID_t &did) const noexcept
//     {
//         auto hval = std::hash<uint32_t>{}(did.ToDWORD());
//         //std::size_t h1 = std::hash<std::string>{}(s.first_name);
//         return hval;
//     }
// };

class PacketDeviceIDStrHelper{
public:
    PacketDeviceIDStrHelper(const PacketDeviceID_t &did){
        HHDeviceAddress_t deva(did);
        deva.ToHexStr(AString, sizeof(AString));
    }

    // PacketDeviceIDStrHelper(const HHDeviceAddress_t &deva){
    //     deva.ToHexStr(AString, sizeof(AString));
    // }

    // const char * c_str(){
    //     AString[HOLYHOP_DID_HEXSTR_SIZE-1]=0; // enforce null-terminated
    //     return AString;
    // }

    operator const HHStrClass() const{
        return HHStrClass(AString);
    }

    // auto format_as(PacketDeviceIDStrHelper &dsh) {
    //      return std::string(AString); //fmt::underlying(f);
    // }

protected:
    char AString[HOLYHOP_DID_HEXSTR_SIZE];
};

#ifndef ARDUINO
// fmt:: helper for PacketDeviceIDStrHelper
template <> struct fmt::formatter<PacketDeviceIDStrHelper>: formatter<string_view> {
    // parse is inherited from formatter<string_view>.
    auto format(PacketDeviceIDStrHelper dev_id, format_context& ctx) const -> format_context::iterator{
    string_view name = std::string(dev_id);
        return formatter<string_view>::format(name, ctx);
    }
};
#endif

template<class TPacket>
int16_t GetPktSize(TPacket &pkt) {
    return sizeof(pkt);
}

// Dont remove from gHHNodes in callbacks
struct HHEvents_t {
#ifdef HOLYHOP_SENSOR
    void (*OnCommandReset)(PacketCommandReset_t *);
    void (*OnCommandGetVersion)(PacketCommandGetVersion_t *);
    void (*OnCommandGetName)(PacketCommandGetName_t *);
    void (*OnCommandSetName)(PacketCommandSetName_t *);
    void (*OnCommandGetInterval)(PacketCommandGetInterval_t *);
    void (*OnCommandSetInterval)(PacketCommandSetInterval_t *);
    void (*OnCommandConfigLoRa)(PacketCommandConfigLoRa_t *);
    void (*OnCommandConfigRadar)(PacketCommandConfigRadar_t *);
    void (*OnCommandGetRadarConfig)(PacketCommandGetRadarConfig_t *);
    void (*OnCommandBeginDFUUpload)(PacketCommandBeginDFUUpload_t *);
    void (*OnCommandDFUUpload)(PacketCommandDFUUpload_t *);
    void (*OnCommandVerifyDFUUpload)(PacketCommandVerifyDFUUpload_t *);
    void (*OnCommandBlinkLED)(PacketCommandBlinkLED_t *);
    void (*OnCommandGetPreferredUplink)(PacketCommandGetPreferredUplink_t *);
    void (*OnCommandSetPreferredUplink)(PacketCommandSetPreferredUplink_t *);

    void (*OnRelayPkt)(PacketHeader_t *ppkt_hdr, uint16_t size);
#endif
#ifdef HOLYHOP_BASE
    void (*OnReplyVersion)(PacketReplyVersion_t *);
    void (*OnReplySuccess)(PacketReplySuccess_t *);
    void (*OnReplyFailure)(PacketReplyFailure_t *);
    void (*OnReplyName)(PacketReplyName_t *);
    void (*OnReplyPreferredUplink)(PacketReplyPreferredUplink_t *);
    void (*OnReplyInterval)(PacketReplyInterval_t *);
    void (*OnReplyRadarConfig)(PacketReplyRadarConfig_t *);
    void (*OnReplyDFUUpload)(PacketReplyDFUUpload_t *);
    void (*OnReplyVerifyDFUUpload)(PacketReplyVerifyDFUUpload_t *);
    // NOTE OnReplyCmd is called before specific Reply handlers
    void (*OnReplyCmd)(const HHDeviceAddress_t &dev_id, PacketType_t reply_type, PacketReplyStatus_t reply_status);
    void (*OnReplyTimeout)(const HHDeviceAddress_t &dev_id);
#endif
    void (*OnTelemetry)(const HHDeviceAddress_t &dev_id, uint8_t base_hops);
    void (*OnRadarTelemetry)(PacketRadarTelemetry_t *);
    void (*OnRadarGNSSTelemetry)(PacketRadarGNSSTelemetry_t *);

    void (*OnNewNode)(const HHDeviceAddress_t &dev_id);
};

void InitHH(bool fbase, const HHDeviceAddress_t &local_dev_id, HHEvents_t &events);
bool ProcessHHPkt(PacketHeader_t *ppkt_hdr, uint16_t size, int16_t this_rssi, int16_t this_snr);
void ReorderHHNodeTlmOffsets();
std::time_t GetHHWakeTime();
uint32_t GetHHWakeElapsed();
uint32_t GetHHWakeNextMS();
uint32_t GetHHWakeInterval();
void SetHHWakeNext( int32_t start_in_sec, uint32_t tlm_int_sec );
void SetHHWakeInterval( uint32_t tlm_int_sec);
HHStrClass GetHHNodeName(const HHDeviceAddress_t &dev_id);
uint32_t GetHHNodeTlmOffsetMs(const HHDeviceAddress_t &dev_id);
//uint32_t GetHHNodeWakeCount(const HHDeviceAddress_t &dev_id);
void SetHHNodeTimedOut(const HHDeviceAddress_t &dev_id);
uint16_t GetHHBaseHops();
HHDeviceAddress_t GetHHUplinkAddr();
HHDeviceAddress_t GetHHPreferredUplinkAddr();
bool GetHHNextHopFor(const PacketDeviceID_t &dest_id, HHDeviceAddress_t &relay_to);
bool ExpireHHOldNodes();
void UpdateHHDatetimes( const std::time_t time_offset );
bool UpdateHHNetworkState();
bool IsHHUplinkValid(); //(const HHDeviceAddress_t &dev_id);
bool SetHHUplink(const HHDeviceAddress_t &dev_id);
const char * GetHHPktTypeName(PacketType_t pkt_type);
const char * GetHHPktReplyStatusName(PacketReplyStatus_t status);

#ifdef HOLYHOP_BASE
const std::vector<HHNodeInfo_t> & GetHHNodes();
HHNodeInfo_t InitHHNodeInfo();
void AddHHNode(const HHNodeInfo_t &node_info);
void QueueHHNodeCmd(HHQueuedCommand_t &qcmd, bool frem_dupes);
bool NextHHQCmd(HHQueuedCommand_t &qnowcmd);
//void OverrideHHCmdQ(const HHDeviceAddress_t &dev_id, const HHQueuedCommand_t &qnowcmd);
void IncrementHHCmdQ(const HHDeviceAddress_t &dev_id);
void RemoveHHQOldCmds();
uint32_t GetHHNodeFWOffset(const HHDeviceAddress_t &dev_id);
void SetHHNodeFWOffset(const HHDeviceAddress_t &dev_id, uint32_t fw_offset);
void ClearHHNodeFWPath(const HHDeviceAddress_t &dev_id );
std::filesystem::path GetHHNodeFWPath(const HHDeviceAddress_t &dev_id);
HHDeviceAddress_t GetHHNodePreferredUplink(const HHDeviceAddress_t &dev_id);
void SetHHNodePreferredUplink(const HHDeviceAddress_t &uplink_id);
bool GetHHFlagResyncAll();
void ClearHHFlagResyncAll();
#endif
