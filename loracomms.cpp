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

// lora comms radio layer & logic
//
#include <atomic>
#include <cstdint>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
#include <thread>
#include <regex>
#include <charconv>
#include <RadioLib.h>
#include "hal/RPi/PiHal.h" // radiolib hardware abstraction layer
#include "holyhop.h"
#include "loracomms.h"
#include "timer.h"
#include "threadsafequeue.h"
#include "forwarddata.h"

#define RF_FREQUENCY 914.100000
#define TX_OUTPUT_POWER 22
#define LORA_BANDWIDTH 125
#define LORA_SPREADING_FACTOR 10
#define LORA_CODINGRATE 5
#define LORA_PREAMBLE_LENGTH 8
#define TCXO_VOLTAGE 1.8f

#define LORA_SPI 0
#define LORA_CS 8       //brown
#define LORA_DIO1 22    //blue
#define LORA_RST 24     //grey
#define LORA_BUSY 23    // purple - SCLK yellow, MOSI white, MISO green

const std::string SENSOR_DATA_DIR("./sensor-data/");
const std::string BASE_NETSTATE_FILENAME("./base_netstate.bin");

const char *RADAR_TELEMETRY_SENTENCEID = "radartlm";
const char *RADAR_CONFIG_SENTENCEID    = "radarcfg";

// helper strings for the status webpage
const char *TELEMETRY_COLUMNS_RADARCFG =
"radarcfg: timestamp,radarcfg,sensor id,sensor name,start_dist_mm,end_dist_mm,max_profile,peak_sorting,\
threshold_method,threshold_sensitivity,fixed_amp_threshold,fixed_str_threshold,\
signal_quality,max_step_count,reflector_shape,close_range_leakage";

const char *TELEMETRY_COLUMNS_RADARTLM =
"radartlm: timestamp,radartlm,sensor id,sensor name,base RSSI,base SNR,base freq offset,\
wake count,batt voltage,sensor RSSI,sensor SNR,\
radar dist0,radar dist1,radar_dist2,radar strength0,\
radarwide_dist0,radarwide_dist1,radarwide_dist2,radarwide_strength0,\
temp C,base hops,uplink";

// the SX1262 CS is connected to CE0
PiHal *hal = new PiHal(LORA_SPI);
SX1262 radio = new Module(hal, LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY );// RADIOLIB_NC);

HHDeviceAddress_t gLocalDID(KnownOUI_t::BASE,0,0,1); // our device id
HHDeviceAddress_t gLastCmdDestID;   // destination id of most recently sent cmd
HHDeviceAddress_t gNewNodeDID;      // tracks the newest node for sync logic in OnRadarTelemetry
uint8_t gLoRaSendBuf[LORA_MAX_PKT_SIZE];
size_t gLoRaSendBufLen = 0;         // if non-zero there is a pkt to send
SimpleTimer gLoRaWaitReply;         // if active we are waiting to RX a packet
SimpleTimer gLoRaPingNodes;         // timer to ping nodes so they stay in sync

// LoRa IRQ event callback
volatile bool gLoRaIrqFlag = false;
void OnRadioInterrupt(void) {
    gLoRaIrqFlag = true;
}

// forward declarations
template<class TPacket>
void QueueHHCmd(TPacket &pkt, const HHDeviceAddress_t &dst_id, bool fsend_now, std::filesystem::path fw_path);
template<class TPacket>
void QueueHHCmd(TPacket &pkt, const HHDeviceAddress_t &dst_id);

// HolyHop Event handlers
void OnHHReplyVersion(PacketReplyVersion_t *);
void OnHHReplySuccess(PacketReplySuccess_t *);
void OnHHReplyFailure(PacketReplyFailure_t *);
void OnHHReplyName(PacketReplyName_t *);
void OnHHReplyPreferredUplink(PacketReplyPreferredUplink_t *);
void OnHHReplyInterval(PacketReplyInterval_t *);
void OnHHReplyRadarConfig(PacketReplyRadarConfig_t *);
void OnHHReplyDFUUpload(PacketReplyDFUUpload_t *);
void OnHHReplyVerifyDFUUpload(PacketReplyVerifyDFUUpload_t *);
void OnHHRadarTelemetry(PacketRadarTelemetry_t *);
void OnHHNewNode( const HHDeviceAddress_t &src_id );
void OnHHReplyCmd( const HHDeviceAddress_t &src_id );

ThreadSafeQueue<HHQueuedCommand_t> gHHCmdQueue;
std::mutex gSharedDataMutex;
std::condition_variable gSharedDataCV;
std::atomic<bool> gfGetNodesMeta(false);
std::atomic<bool> gfReorderTlmOffsets(false);
std::atomic<bool> gfRestartWakeInterval(false);
std::atomic<bool> gfManuallyAddNode(false);
HHDeviceAddress_t gManuallyAddNodeID;       // add this node to array
std::vector<HHNodeMeta_t> gHHNodesMeta;     // web service thread accesses this

std::string CTimeToStr( std::time_t ct ){
    std::tm *ptime = std::localtime(&ct);
    char time_str[std::size("yyyy-mm-ddThh:mm:ss+0100")];
    std::strftime(std::data(time_str), std::size(time_str), "%FT%T%z", ptime);
    return std::string(time_str);
}

void PayloadRadarCfgToRadarCfg( const PayloadReplyRadarConfig_t &payload,
                                RadarConfiguration_t &rcfg )
{
    rcfg.start_dist_mm                  = payload.start_dist_mm;
    rcfg.end_dist_mm                    = payload.end_dist_mm;
    rcfg.max_profile                    = payload.max_profile;
    rcfg.peak_sorting                   = payload.peak_sorting;
    rcfg.threshold_method               = payload.threshold_method;
    rcfg.threshold_sensitivity          = payload.threshold_sensitivity;
    rcfg.fixed_amp_threshold            = payload.fixed_amp_threshold;
    rcfg.fixed_str_threshold            = payload.fixed_str_threshold;
    rcfg.signal_quality                 = payload.signal_quality;
    rcfg.max_step_count                 = payload.max_step_count;
    rcfg.reflector_shape                = payload.reflector_shape;
    rcfg.close_range_leakage            = payload.close_range_leakage;
}

std::string RadarCfgToPrettyStr( const RadarConfiguration_t &rcfg ){
    std::stringstream rcfg_ss;
    rcfg_ss << "start_dist_mm:" <<      rcfg.start_dist_mm << std::endl;
    rcfg_ss << "end_dist_mm:" <<         rcfg.end_dist_mm << std::endl;
    rcfg_ss << "max_profile:" <<         rcfg.max_profile << std::endl;
    rcfg_ss << "peak_sorting:" <<        rcfg.peak_sorting << std::endl;
    rcfg_ss << "threshold_method:" <<    rcfg.threshold_method << std::endl;
    rcfg_ss << "threshold_sensitivity:" << rcfg.threshold_sensitivity << std::endl;
    rcfg_ss << "fixed_amp_threshold:" << rcfg.fixed_amp_threshold << std::endl;
    rcfg_ss << "fixed_str_threshold:" << rcfg.fixed_str_threshold << std::endl;
    rcfg_ss << "signal_quality:" <<      rcfg.signal_quality << std::endl;
    rcfg_ss << "max_step_count:" <<      rcfg.max_step_count << std::endl;
    rcfg_ss << "reflector_shape:" <<     rcfg.reflector_shape << std::endl;
    rcfg_ss << "close_range_leakage:" << (uint32_t)(rcfg.close_range_leakage) << std::endl;
    return rcfg_ss.str();
}

bool GetCSVNodeID( const std::string &str, HHDeviceAddress_t &dev_id ){
    // tokenize CSV
    std::istringstream iss(str);
    std::vector<std::string> substrs;
    std::string token;
    while( std::getline( iss, token, ',' )){
        substrs.push_back(token);
    }

    // get nodeid for known sentence types
    if ( substrs.size() >= 3 &&
         (substrs[1] == RADAR_TELEMETRY_SENTENCEID ||
          substrs[1] == RADAR_CONFIG_SENTENCEID) ){
        std::string nid_str = substrs[2];
        uint32_t dw_id;
        auto [ptr,ec] = std::from_chars(nid_str.data(), nid_str.data() + nid_str.size(), dw_id, 16 );
        if ( ec == std::errc{} ){
            dev_id.SetDW( dw_id );
            // todo: basic dev_id validity check
            return true;
        }
    }

    dev_id.SetNull();
    return false;
}

// Scan sensor log files for nodes we have heard in the past & add the nodes to holyhop array
void ScanLogsForNodes(){
    // open directory to iterate files
    std::filesystem::path sensordata_path(SENSOR_DATA_DIR);
    if ( !std::filesystem::exists(sensordata_path) ){
        TSLogs.Time() << "ScanLogsForNodes " << SENSOR_DATA_DIR << " does not exist. No logs to scan.\n";
        return;
    }

    // Iterate through all files matching the glob
    //const auto glob_pattern = "sensor-data-*.log";
    const std::regex rex("sensor-data-.+\\.log");
    for ( const auto &dir_entry : std::filesystem::directory_iterator(sensordata_path) ){
        if ( std::regex_match( dir_entry.path().filename().string(), rex ) ){
            //TSLogs << "DEBUG It is a match! " << dir_entry.path().filename() << "\n";
            // open file and get all node ids present
            std::ifstream ilog_file(dir_entry.path());
            if (!ilog_file){
                TSLogs.Time() << fmt::format("ERROR could not scan log_file {}\n", dir_entry.path().string());
                continue;
            }

            std::string line_str;
            while (std::getline(ilog_file, line_str)) {
                if (line_str.empty())
                    continue;   // Ignore empty lines
                HHDeviceAddress_t dev_id;
                if ( GetCSVNodeID( line_str, dev_id ) ){
                    HHNodeInfo_t node = InitHHNodeInfo();
                    node.dev_id = dev_id;
                    AddHHNode(node);
                }
            }
            ilog_file.close();
        }
    }
}

const std::string BASE_NETSTATE_VERSION("BNS0"); // netstate file identifier & version

// some templated IO functions
template<class TData>
void twrite( std::ofstream &ofile, TData data_obj ){
    ofile.write( reinterpret_cast<char *>(&data_obj), sizeof(data_obj) );
}

template<>
void twrite<HHStrClass>( std::ofstream &ofile, HHStrClass data_obj ){
    uint16_t slen = std::min( data_obj.length(), (size_t)256 );   // limit to 256 chars
    ofile.write( reinterpret_cast<char *>(&slen), sizeof(slen));
    ofile.write( data_obj.c_str(), data_obj.length() );
}

template<class TData>
void tread( std::ifstream &ifile, TData &data_obj ){
    ifile.read( reinterpret_cast<char *>(&data_obj), sizeof(data_obj) );
}

template<>
void tread<HHStrClass>( std::ifstream &ifile, HHStrClass &data_obj ){
    uint16_t slen = 0;
    ifile.read(reinterpret_cast<char *>(&slen), sizeof(slen));
    char buf[256];
    uint16_t num_to_read = std::min( slen, (uint16_t)sizeof(buf));  // limit to 256 chars
    ifile.read( buf, num_to_read );
    data_obj = std::string(buf, num_to_read);
}

// Save state and node array to file
bool SaveNetworkState(){
    // try to open output file
    std::ofstream ofile(BASE_NETSTATE_FILENAME, std::ios::binary | std::ios::trunc );
    if (!ofile){
        TSLogs.Time() << fmt::format("ERROR could not save {}\n", BASE_NETSTATE_FILENAME );
        return false;
    }
    ofile.write(BASE_NETSTATE_VERSION.data(), BASE_NETSTATE_VERSION.size() );
    twrite(ofile, GetHHWakeTime());
    twrite(ofile, GetHHWakeInterval());

    const std::vector<HHNodeInfo_t> &nodes = GetHHNodes();
    size_t num_nodes = nodes.size();
    twrite(ofile, num_nodes);
    //ofile.write( (char *)num_nodes, sizeof(num_nodes) );
    for ( auto node : nodes ){
        twrite(ofile, node.dev_id.ToDW());
        twrite(ofile, node.protocol_ver);
        twrite(ofile, node.firmware_ver);
        twrite(ofile, node.bootloader_ver);
        twrite(ofile, node.wake_count);
        twrite(ofile, node.last_tx_time);
        twrite(ofile, node.base_hops);
        twrite<HHStrClass>(ofile, node.name);
    }
    return true;
}

// Load state from file
bool LoadNetworkState(){
    // try to open input file
    std::ifstream ifile(BASE_NETSTATE_FILENAME, std::ios::binary);
    if (!ifile){
        TSLogs.Time() << fmt::format("WARNING could not open {}\n", BASE_NETSTATE_FILENAME );
        return false;
    }
    // load & chk id
    char file_idtag[5] = {};
    ifile.read(file_idtag, 4 );
    std::string file_idtag_str( file_idtag );
    if ( file_idtag_str != BASE_NETSTATE_VERSION ){
        TSLogs.Time() << fmt::format("WARNING could not read {}, bad version\n", BASE_NETSTATE_FILENAME );
        return false;
    }
    // load the saved wake time and interval. offset the interval timer to a multiple of saved time.
    // This is important when restarting the server so we can stay in sync with our nodes.
    // Otherwise the network will have to time out and rebuild/resync to a new/different interval.
    std::time_t saved_wake_time;
    tread(ifile, saved_wake_time); // todo: validate
    TSLogs << fmt::format("Loaded wake time {}\n", CTimeToStr(saved_wake_time));

    uint32_t wake_interval_sec;
    tread(ifile, wake_interval_sec); // todo: validate
    TSLogs << fmt::format("Loaded wake interval {} sec\n", wake_interval_sec);

    std::time_t nowt = std::time(nullptr);
    int32_t interval_offset_sec;
    if ( nowt <= saved_wake_time ){
        // saved wake time is in the future
        interval_offset_sec = difftime(saved_wake_time, nowt);
    }
    else{
        // saved wake time is in the past. calculate time to wait for next interval.
        int64_t diff_sec = static_cast<int64_t>(difftime(nowt, saved_wake_time));
        interval_offset_sec = wake_interval_sec - (diff_sec % wake_interval_sec);
        TSLogs << fmt::format("Calculated next wake will be in {} sec, diff_sec {}\n", interval_offset_sec, diff_sec );
    }
    SetHHWakeNext( interval_offset_sec, wake_interval_sec );

    // Load Nodes
    size_t num_nodes;
    tread(ifile, num_nodes);
    //ofile.write( (char *)num_nodes, sizeof(num_nodes) );
    for (size_t i=0; i<num_nodes; i++){
        HHNodeInfo_t node = InitHHNodeInfo();
        uint32_t dw_id;
        tread(ifile, dw_id);
        node.dev_id.SetDW(dw_id);
        tread(ifile, node.protocol_ver);
        tread(ifile, node.firmware_ver);
        tread(ifile, node.bootloader_ver);
        tread(ifile, node.wake_count);
        tread(ifile, node.last_tx_time);
        tread(ifile, node.base_hops);
        tread<HHStrClass>(ifile, node.name);

        AddHHNode( node );
    }
    return true;
}

// Log sensor telemetry and whatever else to named textfiles
void LogSensorData( const char *astr ){
    // figure out the time
    std::time_t tnow = std::time(nullptr);
    std::tm *pnow = std::localtime(&tnow);
    char time_str[std::size("yyyy-mm-ddThh:mm:ss+0100,")];
    std::strftime(std::data(time_str), std::size(time_str), "%FT%T%z,", pnow);
    int year = pnow->tm_year + 1900;
    int month = pnow->tm_mon + 1;
    int day = pnow->tm_mday;

    // creeate sensor data log dir if needed
    std::filesystem::path sensordata_path(SENSOR_DATA_DIR);
    if ( !std::filesystem::exists(sensordata_path) ){
        TSLogs.Time() << SENSOR_DATA_DIR << " does not exist. creating directory.\n";
        std::error_code ec;
        if ( !std::filesystem::create_directories(sensordata_path, ec) ){
            TSLogs.Time() << fmt::format("ERROR could not create sensor data directory: {} {}\n",
                                  ec.value(), ec.message());
            return;
        }
    }

    // open log and append entry
    char log_fname[256];
    snprintf(log_fname,sizeof(log_fname),"sensor-data-%04d-%02d-%02d.log",year,month,day);
    std::filesystem::path log_fullpath = sensordata_path / std::string(log_fname);

    std::ofstream log_file(log_fullpath, std::ios::app);
    if (!log_file){
        TSLogs.Time() << fmt::format("ERROR could not open log_file {}\n", log_fullpath.string());
        return;
    }
    log_file << time_str << astr << std::endl;
    log_file.close();
}

// A command reply matching dev_id's last sent command ref_id arrived.
void OnHHReplyCmd( const HHDeviceAddress_t &dev_id, PacketType_t reply_type, PacketReplyStatus_t reply_status ){
    TSLogs.Time() << "OnHHReplyCmd\n";
    // sanity check it is from the correct node
    if ( dev_id == gLastCmdDestID )
        gLoRaWaitReply.stop(); // stop wait timer
    else
    {
        TSLogs.Time() << fmt::format("ERROR reply is from wrong node 0x{}, expected 0x{}\n",
                                     dev_id, gLastCmdDestID );;
    }
}

void OnHHReplyTimeout(const HHDeviceAddress_t &dev_id){
    TSLogs.Time() << "OnHHReplyTimeout\n";
    gLoRaWaitReply.stop(); // stop wait timer
}

void OnHHReplyVersion(PacketReplyVersion_t *ppkt){
    TSLogs.Time() << fmt::format("OnHHReplyVersion hh_version 0x{:04X} fw_version 0x{:04X} bl_version 0x{:08X}\n",
        ppkt->Payload.hh_version, ppkt->Payload.fw_version, ppkt->Payload.bl_version);

    if ( ppkt->Payload.hh_version != HOLYHOP_VERSION ){
        PacketDeviceIDStrHelper src_id_str(ppkt->Header.Src);
        TSLogs.Time() << fmt::format("Warning! HolyHop version mismatch. Our version 0x{:04X}. ", HOLYHOP_VERSION);
        TSLogs.Time() << fmt::format("Node {} version 0x{:04X}.\n", src_id_str, ppkt->Payload.hh_version);
    }
}

void OnHHReplySuccess(PacketReplySuccess_t *){
    TSLogs.Time() << "OnHHReplySuccess\n";
}

void OnHHReplyFailure(PacketReplyFailure_t *){
    TSLogs.Time() << "OnHHReplyFailure\n";
}

void OnHHReplyName(PacketReplyName_t *ppkt){
    PacketDeviceIDStrHelper src_id_str(ppkt->Header.Src);
    std::string node_name = ppkt->Payload.name.Get();
    TSLogs.Time() << fmt::format("OnHHReplyName node {} name {}\n", src_id_str, node_name);
}

void OnHHReplyPreferredUplink(PacketReplyPreferredUplink_t *ppkt){
    PacketDeviceIDStrHelper src_id_str(ppkt->Header.Src);
    uint32_t prf_uplink_id = ppkt->Payload.uplink;
    TSLogs.Time() << fmt::format("OnHHReplyPreferredUplink node {} preferred uplink {}\n",
                                    src_id_str, HHDeviceAddress_t(prf_uplink_id));
}

void OnHHReplyInterval(PacketReplyInterval_t *ppkt){
    PacketDeviceIDStrHelper src_id_str(ppkt->Header.Src);
    TSLogs.Time() << fmt::format("OnHHReplyInterval node {} wake_sec {} measurement_ms {}\n",
        src_id_str, ppkt->Payload.tlm_int_sec, ppkt->Payload.measurement_ms);
}

// The send interval pkt payload is timing sensitive. Fill it out.
void UpdateCmdSendIntervalPkt( PacketCommandSetInterval_t &pkt_cmd_int, const HHDeviceAddress_t &dev_id ){
    //todo: recalc tlm_offset_ms if node mesh location/hops change
    pkt_cmd_int.Payload.measurement_ms  = GetHHWakeInterval() * 1000; //HOLYHOP_DEFAULT_TLM_INTERVAL_SEC * 1000;
    pkt_cmd_int.Payload.tlm_int_sec     = GetHHWakeInterval();
    pkt_cmd_int.Payload.next_tlm_ms     = GetHHWakeNextMS();
    pkt_cmd_int.Payload.tlm_offset_ms   = GetHHNodeTlmOffsetMs(dev_id);
    pkt_cmd_int.Payload.etime_sec       = std::time(nullptr); // current time in epoch sec
}

// @brief synchronize node by sending setinterval command. Normally SENDNOW.
// But can be queued for later as well.
void CmdNodeToSync( const HHDeviceAddress_t &dev_id, bool fsend_now=true ){
    TSLogs.Time() << "CmdNodeToSync() " << dev_id << "\n";
    PacketCommandSetInterval_t pkt_cmd_int;
    UpdateCmdSendIntervalPkt( pkt_cmd_int, dev_id );
    QueueHHCmd(pkt_cmd_int, dev_id, fsend_now, {});
}

// @brief ask remote node for its info and sync it with setinterval
void QueryNodeInfo( const HHDeviceAddress_t &dev_id ){
    TSLogs.Time() << fmt::format("Query node info: {}\n", dev_id);
    // ask node for its version, name, prf uplink
    //todo: consolidate these into one cmd/reply in new protocol version?
    PacketCommandGetVersion_t pkt_cmd_ver;
    QueueHHCmd(pkt_cmd_ver, dev_id );

    PacketCommandGetName_t pkt_cmd_name;
    QueueHHCmd(pkt_cmd_name, dev_id);

    PacketCommandGetPreferredUplink_t pkt_cmd_prfupl;
    QueueHHCmd(pkt_cmd_prfupl, dev_id);
}

void OnHHReplyRadarConfig( PacketReplyRadarConfig_t *ppkt ){
    HHDeviceAddress_t src_id(ppkt->Header.Src);
    RadarConfiguration_t rcfg;
    PayloadRadarCfgToRadarCfg(ppkt->Payload, rcfg);
    std::string node_name = GetHHNodeName(src_id);

    // print to console log
    std::stringstream console_fmt_ss;
    console_fmt_ss << "radarcfg: src_id {}, name {}, start_dist_mm {},"
        << " end_dist_mm {}, max_profile {}, peak_sorting {}, threshold_method {},"
        << " threshold_sensitivity {}, fixed_amp_threshold {}, fixed_str_threshold {},"
        << " signal_quality {}, max_step_count {}, reflector_shape {}, close_range_leakage {}\n";
    TSLogs.Time() << fmt::format( console_fmt_ss.str(),
        src_id, node_name, rcfg.start_dist_mm, rcfg.end_dist_mm, rcfg.max_profile,
        rcfg.peak_sorting, rcfg.threshold_method, rcfg.threshold_sensitivity,
        rcfg.fixed_amp_threshold, rcfg.fixed_str_threshold, rcfg.signal_quality,
        rcfg.max_step_count, rcfg.reflector_shape, rcfg.close_range_leakage
    );

    // log to file
    std::string tstr = fmt::format("radarcfg,{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        src_id, node_name, rcfg.start_dist_mm, rcfg.end_dist_mm, rcfg.max_profile,
        rcfg.peak_sorting, rcfg.threshold_method, rcfg.threshold_sensitivity,
        rcfg.fixed_amp_threshold, rcfg.fixed_str_threshold, rcfg.signal_quality,
        rcfg.max_step_count, rcfg.reflector_shape, rcfg.close_range_leakage
    );
    LogSensorData( tstr.c_str() );
}

// RETURNS: size of device's current fw upload file
std::streamsize GetFWFileSize( const HHDeviceAddress_t &dev_id){
    // open fw file
    std::filesystem::path fw_path = GetHHNodeFWPath(dev_id);
    std::ifstream fw_file( fw_path, std::ios::binary );
    if ( !fw_file ){
        TSLogs.Time() << fmt::format("GetFWFileSize failed could not open {}!\n",
                                     fw_path.filename().string() );
        return 0;
    }

    fw_file.seekg(0, std::ios_base::end);
    std::streamsize fw_size = fw_file.tellg();
    if ( fw_file.fail() ){
        TSLogs.Time() << fmt::format("GetFWFileSize failed could not seekg {} to end!\n",
                                     fw_path.filename().string() );
        return 0;
    }

    return fw_size;
}

void OnHHReplyDFUUpload( PacketReplyDFUUpload_t *ppkt ){
    HHDeviceAddress_t src_id(ppkt->Header.Src);
    SetHHNodeFWOffset(src_id, ppkt->Payload.current_offset);    // use this as current offset

    // dfu update began or continues.. success?
    if ( !ppkt->Payload.fsuccess ){
        TSLogs.Time() << "DFU Upload failed at offset " << ppkt->Payload.current_offset << "\n";
        return;
    }

    // success. do next chunk or if done verify
    std::streamsize fw_size = GetFWFileSize(src_id);
    size_t fw_offset = GetHHNodeFWOffset(src_id);
    TSLogs << "DEBUG OnHHReplyDFUUpload fw_size " << fw_size << ", fw_offset " << fw_offset << "\n";

    // are we done?
    std::streamsize fw_size_left = fw_size - fw_offset;
    if ( fw_size_left <= 0){
        TSLogs.Time() << fmt::format("DFU Upload done ({} bytes remain)\n", fw_size_left);

        PacketCommandVerifyDFUUpload_t pkt_vdfu;
        bool fsend_now = true; // ..send now to finish the DFU. dest node should still be listening
        QueueHHCmd(pkt_vdfu, src_id, fsend_now, {});
        return;
    }

    // queue cmd w/o fw data. packet is built after being popped from command queue.
    PacketCommandDFUUpload_t pkt_dfu;
    bool fsend_now = true;  // we successfully started/resumed fw update so
                            // ..send now true to continously stream the fw chunks
    QueueHHCmd(pkt_dfu, src_id, fsend_now, {});
}

// @brief build command packet to upload next chunk of firmware to node
void BuildDFUUploadCmd( HHQueuedCommand_t &qcmd ){ //const HHDeviceAddress_t &dev_id, PacketCommandDFUUpload_t &pkt_dfu ){
    uint8_t dfu_chunk_size = sizeof(PayloadDFUUpload_t::barray);
    HHDeviceAddress_t &dev_id = qcmd.dest_id;
    PacketCommandDFUUpload_t &pkt_dfu = *reinterpret_cast<PacketCommandDFUUpload_t *>(qcmd.cmd_pkt_buf.data());

    // open fw file
    std::filesystem::path fw_path = GetHHNodeFWPath(dev_id);
    std::ifstream fw_file( fw_path, std::ios::binary );
    if ( !fw_file ){
        TSLogs.Time() << fmt::format("DFU Upload failed could not open {}\n",
                                     fw_path.filename().string() );
        return;
    }

    // get file size then seek to current offset
    std::streamsize fw_offset = GetHHNodeFWOffset(dev_id);
    fw_file.seekg(0, std::ios_base::end);
    std::streamsize fw_size = fw_file.tellg();
    fw_file.seekg(fw_offset);
    TSLogs << "DEBUG BuildDFUUploadCmd fw_size " << fw_size << ", fw_offset " << fw_offset << ", tellg " << fw_file.tellg() << "\n";

    if ( fw_file.fail() ){
        TSLogs.Time() << fmt::format("DFU Upload failed could not seekg {} to {}\n",
                                     fw_path.filename().string(), fw_offset );
        return;
    }

    // are we done?
    int32_t fw_size_left = fw_size - fw_offset;
    if ( fw_size_left <= 0){
        pkt_dfu.Payload.dfu_offset = 0;
        pkt_dfu.Payload.num_bytes = 0;
        TSLogs.Time() << fmt::format("DFU Upload already done! ({} bytes remain)\n", fw_size_left);
        return;
    }

    // calc. next chunk size (the final chunk could be smaller)
    uint8_t this_chunk_size = std::min( fw_size_left, (int32_t)dfu_chunk_size );
    TSLogs << "DEBUG fw_size_left " << fw_size_left << ", this_chunk_size " << (int32_t)this_chunk_size << "\n";

    // read & send next fw file chunk
    if ( !fw_file.read(reinterpret_cast<char*>(pkt_dfu.Payload.barray), this_chunk_size) ){
        TSLogs.Time() << fmt::format("DFU Upload failed could not read {} at {}\n",
                                     fw_path.filename().string(), fw_offset );
        return;
    }
    pkt_dfu.Payload.dfu_offset = fw_offset;
    pkt_dfu.Payload.num_bytes = this_chunk_size;
    TSLogs.Time() << fmt::format("$$$ DFU uploading chunk to {} offset {} size {}\n", dev_id, fw_offset, this_chunk_size);
}

void OnHHReplyVerifyDFUUpload( PacketReplyVerifyDFUUpload_t *ppkt ){
    HHDeviceAddress_t src_id(ppkt->Header.Src);

    // is node even doing an update right now?
    if ( GetHHNodeFWPath(src_id).empty() ){
        TSLogs.Time() << fmt::format("Received PacketReplyVerifyDFUUpload but {} is not doing DFU! Ignoring.\n",
                                      src_id);
        return;
    }

    // dfu verify success?
    if ( !ppkt->Payload.fsuccess ){
        TSLogs.Time() << fmt::format("DFU Verify failed for {}\n", src_id);
        return;
    }

    TSLogs.Time() << fmt::format("$$$ DFU upload verified on {}, ready to reboot sensor\n", src_id);
    ClearHHNodeFWPath(src_id);
    TSLogs.Time() << fmt::format("Rebooting sensor for DFU {}\n", src_id);
    PacketCommandReset_t pkt;
    QueueHHCmd(pkt, src_id);
}

void OnHHRadarTelemetry(PacketRadarTelemetry_t *ppkt){
    HHDeviceAddress_t src_id(ppkt->Header.Src);
    uint8_t base_hops = ppkt->BaseHops;
    PayloadRadarTelemetry_t &payload = ppkt->Payload;
    // scale and fix radar telemetry.
    // convert strength from dB x 100 to dB x 1000 for historical data consistency
    int32_t radar_strength0 = payload.radar_strength0 * 10;
    int32_t radarwide_strength0 = payload.radarwide_strength0 * 10;

    float local_rssi = radio.getRSSI();
    float local_snr = radio.getSNR();
    float local_freq_off = radio.getFrequencyError();
    std::string node_name = GetHHNodeName(src_id);
    std::string uplink_str = fmt::format("{:08X}", payload.uplink);

    // print to console log with labels because its convienent for debugging
    TSLogs.Time() << fmt::format(
        "radartlm: src_id {}, name {}, local_rssi {: .2f}, local_snr {: .2f}, local_freq_off {: .2f}, wake_count {}, batt_voltage {}, src_last_rssi {}, src_last_snr {}, radar_dist0 {}, radar_dist1 {}, radar_dist2 {}, radar_strength0 {}, radarwide_dist0 {}, radarwide_dist1 {}, radarwide_dist2 {}, radarwide_strength0 {}, temp_c {}, base_hops {}, uplink {}\n",
        src_id, node_name, local_rssi, local_snr, local_freq_off, payload.wake_count, payload.batt_voltage,
        payload.last_rssi, payload.last_snr,
        payload.radar_dist0, payload.radar_dist1, payload.radar_dist2, radar_strength0,
        payload.radarwide_dist0, payload.radarwide_dist1, payload.radarwide_dist2, radarwide_strength0,
        payload.temp_c, base_hops, uplink_str
    );

    // log to file
    char tstr[1024];
    snprintf(tstr,sizeof(tstr),"radartlm,%s,%s,%0.2f,%0.2f,%0.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s",
             std::string(src_id).c_str(), node_name.c_str(), local_rssi, local_snr, local_freq_off, payload.wake_count,
             payload.batt_voltage, payload.last_rssi, payload.last_snr,
             payload.radar_dist0, payload.radar_dist1, payload.radar_dist2, radar_strength0,
             payload.radarwide_dist0, payload.radarwide_dist1, payload.radarwide_dist2, radarwide_strength0,
             payload.temp_c, base_hops, uplink_str.c_str()
    );
    LogSensorData( tstr );

    // queue data for forwarding to cloud services
    auto nowt = std::chrono::system_clock::now();   // get epoch time in ms
    int64_t timestamp_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(nowt.time_since_epoch()).count();
    TSLogs << "timestamp_ms: " << timestamp_ms << "\n";
    json sdata;
    sdata["ts"] = timestamp_ms;
    sdata["values"]["timestamp"] = TeeStream::TimeStamp();
    sdata["values"]["sensor_id"] = src_id.c_str();
    sdata["values"]["sensor_name"] = node_name;
    sdata["values"]["base_rssi"] = local_rssi;
    sdata["values"]["base_snr"] = local_snr;
    sdata["values"]["base_freq_off"] = local_freq_off;
    sdata["values"]["wake_count"] = payload.wake_count;
    sdata["values"]["batt_voltage"] = payload.batt_voltage;
    sdata["values"]["last_rssi"] = payload.last_rssi;
    sdata["values"]["last_snr"] = payload.last_snr;
    sdata["values"]["radar_dist0"] = payload.radar_dist0;
    sdata["values"]["radar_dist1"] = payload.radar_dist1;
    sdata["values"]["radar_dist2"] = payload.radar_dist2;
    sdata["values"]["radar_strength0"] = radar_strength0;
    sdata["values"]["radarwide_dist0"] = payload.radarwide_dist0;
    sdata["values"]["radarwide_dist1"] = payload.radarwide_dist1;
    sdata["values"]["radarwide_dist2"] = payload.radarwide_dist2;
    sdata["values"]["radarwide_strength0"] = radarwide_strength0;
    sdata["values"]["temp_c"] = payload.temp_c;
    sdata["values"]["base_hops"] = base_hops;
    sdata["values"]["uplink"] = uplink_str;
    std::string json_str = sdata.dump();
    QueueSensorDataPost( src_id, json_str );

    // Check if node is in sync and info is up to date..
    //
    // if this IS NOT a NEW NODE..
    if (src_id != gNewNodeDID){
        // If node is outside of the RX window or it is not in sync yet, sync it.
        bool fin_sync = base_hops >= 1;
        bool foutside_rxw = GetHHWakeElapsed() > HOLYHOP_RX_WINDOW_SEC;
        if ( !fin_sync || foutside_rxw ){
            TSLogs.Time() << fmt::format("Node telemetry is out of sync: {}\n", src_id);
            CmdNodeToSync(src_id);
            // todo: rate limit these kind of comm failures
        }

        // if wake count is 0, node might have rebooted (or wake count rolled over).
        // lets ask it for its latest info.
        if ( payload.wake_count == 0 ){
            TSLogs.Time() << fmt::format("Node may have rebooted. Wake count 0: {}\n", src_id);
            QueryNodeInfo(src_id);
        }
    }
    // else this is a first telemetry packet from a new node. Synchronizing it would be redundant.
    else {
        gNewNodeDID.SetNull(); // no longer a new node.
    }
}
/*
void OnHHRadarGNSSTelemetry(PacketRadarGNSSTelemetry_t *ppkt){
    HHDeviceAddress_t src_id(ppkt->Header.Src);
    uint8_t base_hops = ppkt->BaseHops;
    PayloadRadarGNSSTelemetry_t &payload = ppkt->Payload;
    // scale and fix radar telemetry.
    // convert strength from dB x 100 to dB x 1000 for historical data consistency
    int32_t radar_strength0 = payload.radar_strength0 * 10;
    int32_t radar_strength1 = payload.radar_strength1 * 10;
    int32_t radar_strength2 = payload.radar_strength2 * 10;
    int32_t radar_strength3 = payload.radar_strength3 * 10;
    int32_t radar_strength4 = payload.radar_strength4 * 10;
    int32_t radar_strength5 = payload.radar_strength5 * 10;
    int32_t radar_strength6 = payload.radar_strength6 * 10;
    int32_t radar_strength7 = payload.radar_strength7 * 10;
    int32_t radar_strength8 = payload.radar_strength8 * 10;
    int32_t radar_strength9 = payload.radar_strength9 * 10;
    // convert lat, long, alt to meters
    double latitude = payload.latitude / 10000000.0f;
    double longitude = payload.longitude / 10000000.0f;
    double altitudem = payload.altitude / 1000.0f;

    float local_rssi = radio.getRSSI();
    float local_snr = radio.getSNR();
    float local_freq_off = radio.getFrequencyError();
    std::string node_name = GetHHNodeName(src_id);
    std::string uplink_str = fmt::format("{:08X}", payload.uplink);

    // print to console log with labels because its convienent for debugging
    TSLogs.Time() << fmt::format(
        "radartlm: src_id {}, name {}, local_rssi {: .2f}, local_snr {: .2f}, local_freq_off {: .2f}, wake_count {}, batt_voltage {}, src_last_rssi {}, src_last_snr {}, radar_dist0 {}, radar_dist1 {}, radar_dist2 {}, radar_strength0 {}, radarwide_dist0 {}, radarwide_dist1 {}, radarwide_dist2 {}, radarwide_strength0 {}, temp_c {}, base_hops {}, uplink {}, latitude {:f}, longitude {:f}, altitude {:f}, fix_type {}\n",
           src_id, node_name, local_rssi, local_snr, local_freq_off, payload.wake_count, payload.batt_voltage,
           payload.last_rssi, payload.last_snr,
           payload.radar_dist0, radar_strength0,
           payload.radar_dist1, radar_strength1,
           payload.radar_dist2, radar_strength2,
           payload.radar_dist3, radar_strength3,
           payload.radar_dist4, radar_strength4,
           payload.radar_dist5, radar_strength5,
           payload.radar_dist6, radar_strength6,
           payload.radar_dist7, radar_strength7,
           payload.radar_dist8, radar_strength8,
           payload.radar_dist9, radar_strength9,
           payload.temp_c, base_hops, uplink_str,
           latitude, longitude, altitudem, payload.fix_type
    );

    // log to file
    char tstr[1024];
    snprintf(tstr,sizeof(tstr),"radartlm,%s,%s,%0.2f,%0.2f,%0.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%f,%f,%f,%d",
        std::string(src_id).c_str(), node_name.c_str(), local_rssi, local_snr, local_freq_off, payload.wake_count,
        payload.batt_voltage, payload.last_rssi, payload.last_snr,
        payload.radar_dist0, radar_strength0,
        payload.radar_dist1, radar_strength1,
        payload.radar_dist2, radar_strength2,
        payload.radar_dist3, radar_strength3,
        payload.radar_dist4, radar_strength4,
        payload.radar_dist5, radar_strength5,
        payload.radar_dist6, radar_strength6,
        payload.radar_dist7, radar_strength7,
        payload.radar_dist8, radar_strength8,
        payload.radar_dist9, radar_strength9,
        payload.temp_c, base_hops, uplink_str.c_str(),
        latitude, longitude, altitudem, payload.fix_type
    );
    LogSensorData( tstr );

    // queue data for forwarding to cloud services
    auto nowt = std::chrono::system_clock::now();   // get epoch time in ms
    int64_t timestamp_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(nowt.time_since_epoch()).count();
    TSLogs << "timestamp_ms: " << timestamp_ms << "\n";
    json sdata;
    sdata["ts"] = timestamp_ms;
    sdata["values"]["timestamp"] = TeeStream::TimeStamp();
    sdata["values"]["sensor_id"] = src_id.c_str();
    sdata["values"]["sensor_name"] = node_name;
    sdata["values"]["base_rssi"] = local_rssi;
    sdata["values"]["base_snr"] = local_snr;
    sdata["values"]["base_freq_off"] = local_freq_off;
    sdata["values"]["wake_count"] = payload.wake_count;
    sdata["values"]["batt_voltage"] = payload.batt_voltage;
    sdata["values"]["last_rssi"] = payload.last_rssi;
    sdata["values"]["last_snr"] = payload.last_snr;
    sdata["values"]["radar_dist0"] = payload.radar_dist0;
    sdata["values"]["radar_strength0"] = radar_strength0;
    sdata["values"]["radar_dist1"] = payload.radar_dist1;
    sdata["values"]["radar_strength1"] = radar_strength1;
    sdata["values"]["radar_dist2"] = payload.radar_dist2;
    sdata["values"]["radar_strength2"] = radar_strength2;
    sdata["values"]["radar_dist3"] = payload.radar_dist3;
    sdata["values"]["radar_strength3"] = radar_strength3;
    sdata["values"]["radar_dist4"] = payload.radar_dist4;
    sdata["values"]["radar_strength4"] = radar_strength4;
    sdata["values"]["radar_dist5"] = payload.radar_dist5;
    sdata["values"]["radar_strength5"] = radar_strength5;
    sdata["values"]["radar_dist6"] = payload.radar_dist6;
    sdata["values"]["radar_strength6"] = radar_strength6;
    sdata["values"]["radar_dist7"] = payload.radar_dist7;
    sdata["values"]["radar_strength7"] = radar_strength7;
    sdata["values"]["radar_dist8"] = payload.radar_dist8;
    sdata["values"]["radar_strength8"] = radar_strength8;
    sdata["values"]["radar_dist9"] = payload.radar_dist9;
    sdata["values"]["radar_strength9"] = radar_strength9;
    sdata["values"]["temp_c"] = payload.temp_c;
    sdata["values"]["base_hops"] = base_hops;
    sdata["values"]["uplink"] = uplink_str;
    sdata["values"]["latitude"] = latitude;
    sdata["values"]["longitude"] = longitude;
    sdata["values"]["altitude"] = altitudem;
    sdata["values"]["fix_type"] = payload.fix_type;
    std::string json_str = sdata.dump();
    QueueSensorDataPost( src_id, json_str );

    // Check if node is in sync and info is up to date..
    //
    // if this IS NOT a NEW NODE..
    if (src_id != gNewNodeDID){
        // If node is outside of the RX window or it is not in sync yet, sync it.
        bool fin_sync = base_hops >= 1;
        bool foutside_rxw = GetHHWakeElapsed() > HOLYHOP_RX_WINDOW_SEC;
        if ( !fin_sync || foutside_rxw ){
            TSLogs.Time() << fmt::format("Node telemetry is out of sync: {}\n", src_id);
            CmdNodeToSync(src_id);
            // todo: rate limit these kind of comm failures
        }

        // if wake count is 0, node might have rebooted (or wake count rolled over).
        // lets ask it for its latest info.
        if ( payload.wake_count == 0 ){
            TSLogs.Time() << fmt::format("Node may have rebooted. Wake count 0: {}\n", src_id);
            QueryNodeInfo(src_id);
        }
    }
    // else this is a first telemetry packet from a new node. Synchronizing it would be redundant.
    else {
        gNewNodeDID.SetNull(); // no longer a new node.
    }
}
*/

void OnHHNewNode( const HHDeviceAddress_t &dev_id ){
    TSLogs.Time() << fmt::format("New node: {}\n", dev_id);
    // sync & ask node for its version, name
    CmdNodeToSync(dev_id);
    QueryNodeInfo(dev_id);
    gNewNodeDID = dev_id;
}

void SendBufferedLoRaPacket(){
    PacketHeader_t *ppkt_hdr = reinterpret_cast<PacketHeader_t *>(gLoRaSendBuf);

    PacketDeviceIDStrHelper dest_addr(ppkt_hdr->Dest);
    PacketDeviceIDStrHelper src_addr(ppkt_hdr->Src);
    PacketDeviceIDStrHelper relayby_addr(ppkt_hdr->RelayBy);
    PacketDeviceIDStrHelper relayto_addr(ppkt_hdr->RelayTo);

    TSLogs.Time() << fmt::format("TX LoRa packet Src {}, Dest {}, RelayBy {}, RelayTo {}\n",
            src_addr, dest_addr, relayby_addr, relayto_addr );
    TSLogs << fmt::format("Packet Type: ({}) {}, {} hops, size {}, time on air {}ms\n",
            ppkt_hdr->PktType, GetHHPktTypeName(ppkt_hdr->PktType), ppkt_hdr->HopCount,
            gLoRaSendBufLen, radio.getTimeOnAir(gLoRaSendBufLen)/1000);

    // set interval command packets are timing sensitive...
    if ( ppkt_hdr->PktType == PacketType_t::CMD_SET_INTERVAL ){
        // rebuild the packet now with latest times
        PacketCommandSetInterval_t *ppkt_si = reinterpret_cast<PacketCommandSetInterval_t *>(gLoRaSendBuf);
        HHDeviceAddress_t dest_id(ppkt_hdr->Dest);
        UpdateCmdSendIntervalPkt( *ppkt_si, dest_id );

        TSLogs.Time() << fmt::format(
            "Update Set Interval Pkt measure {}ms tlm_int {}s next_tlm {}ms tlm_off {}ms local_time {}\n",
            ppkt_si->Payload.measurement_ms, ppkt_si->Payload.tlm_int_sec,
            ppkt_si->Payload.next_tlm_ms, ppkt_si->Payload.tlm_offset_ms, ppkt_si->Payload.etime_sec );
    }

    int rstate = radio.startTransmit(gLoRaSendBuf, gLoRaSendBufLen);
    if (rstate != RADIOLIB_ERR_NONE){
        TSLogs.Time() << fmt::format("TX failed, code {}\n", rstate);
    }
}

void SendLoRaPacket(std::array<uint8_t,LORA_MAX_PKT_SIZE> &pkt_array, size_t pkt_size, const HHDeviceAddress_t &dst_id) {
    // stop RX reply wait timer in case it was running
    gLoRaWaitReply.stop();

    // copy to our send buffer
    std::copy(pkt_array.begin(),pkt_array.end(),gLoRaSendBuf);
    gLoRaSendBufLen = pkt_size;  // if non-zero there is a pkt to send
    gLastCmdDestID = dst_id;

    PacketHeader_t *ppkt_hdr = reinterpret_cast<PacketHeader_t *>(gLoRaSendBuf);
    // set packet destination & source ids & relay
    ppkt_hdr->Dest = dst_id.ToDID();
    ppkt_hdr->Src = gLocalDID.ToDID();      // from us
    ppkt_hdr->RelayBy = gLocalDID.ToDID();  // direct from us
    ppkt_hdr->HopCount = 1;        // this is it's first hop

    // check our nodeinfo data to see if we need relay to this Destination.
    HHDeviceAddress_t relay_to;
    if ( GetHHNextHopFor( dst_id.ToDID(), relay_to ) )
        ppkt_hdr->RelayTo = relay_to.ToDID();
    else
        ppkt_hdr->RelayTo = dst_id.ToDID(); // we don't know how to reach. try direct.

    // HHDeviceAddress_t dsta_id(pkt.Header.Dest);
    // HHDeviceAddress_t srca_id(pkt.Header.Src);
    // TSLogs.Time() << "Send LoRa packet to 0x" << (const char *)dsta_id << " from us 0x"
    //     << (const char *)srca_id << " type (" << (uint)pkt.Header.PktType << ")"
    //     << GetHHPktTypeName(pkt.Header.PktType) << " size " << pkt_size << "\n";

    // if ( !relay_to.IsNull() )
    //     TSLogs.Fmt("Relay To 0x%s\n", relay_to.c_str());

    // do radio RX->TX turnaround delay (so other nodes have time to TX->RX)
    std::this_thread::sleep_for(std::chrono::milliseconds(LORA_TURNAROUND_DELAYMS));

    // todo: we could do LoRa ChannelActivityDetection to see if channel is clear right now
    // StartLoRaCAD();
    SendBufferedLoRaPacket();
}

///////////////////////////////////////////////////////////////

// On EXIT free resources
void EXITHandler(){
    TSLogs.Time() << "Exiting...";
    // save network state to disk
    if ( !SaveNetworkState() ){
        TSLogs << "\n";
        TSLogs.Time() << "ERROR failed to save network state\n";
    }
    TSLogs << "done.\n";
}

void InitLoRa(){
    // setup holyhop
    HHEvents_t hh_events = {};
    hh_events.OnReplyVersion            = OnHHReplyVersion;
    hh_events.OnReplySuccess            = OnHHReplySuccess;
    hh_events.OnReplyFailure            = OnHHReplyFailure;
    hh_events.OnReplyName               = OnHHReplyName;
    hh_events.OnReplyPreferredUplink    = OnHHReplyPreferredUplink;
    hh_events.OnReplyInterval           = OnHHReplyInterval;
    hh_events.OnReplyRadarConfig        = OnHHReplyRadarConfig;
    hh_events.OnReplyDFUUpload          = OnHHReplyDFUUpload;
    hh_events.OnReplyVerifyDFUUpload    = OnHHReplyVerifyDFUUpload;
    hh_events.OnRadarTelemetry          = OnHHRadarTelemetry;
    hh_events.OnNewNode                 = OnHHNewNode;
    hh_events.OnReplyCmd                = OnHHReplyCmd;
    hh_events.OnReplyTimeout            = OnHHReplyTimeout;
    InitHH( true, gLocalDID, hh_events );

    // load state from disk
    TSLogs.Time() << "Loading saved network state from disk\n";
    //ScanLogsForNodes();
    if ( !LoadNetworkState() ){
        TSLogs << "failed!\n";
    }

    // trap EXIT to save network state & close resources
    std::atexit(EXITHandler);

    // initialize LoRa radio
    TSLogs.Time() << fmt::format("[SX1262] Initializing... ");
    int rstate = radio.begin(RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, TX_OUTPUT_POWER, LORA_PREAMBLE_LENGTH, TCXO_VOLTAGE);

    if (rstate != RADIOLIB_ERR_NONE) {
        TSLogs.Time() << fmt::format("failed, code {}\n", rstate);
        exit(-1);
        return;
    }
    TSLogs << "success!\n";

    radio.setDio2AsRfSwitch(true);
    radio.setRxBoostedGainMode(true);
    // state = radio.setTCXO( 1.8f, 5000 );
    radio.setDio1Action(OnRadioInterrupt);

    // seed our RNG from the radio noise
    int32_t rseed = radio.random(std::numeric_limits<int32_t>::max());
    srand( rseed );
    TSLogs.Time() << fmt::format("random seed from radio: {}\n", rseed);

    rstate = radio.startReceive();
    TSLogs.Time() << fmt::format("[SX1262] Listening...");
    if (rstate != RADIOLIB_ERR_NONE) {
        TSLogs.Time() << fmt::format("failed, code {}\n", rstate);
        exit(-1);
        return;
    }
    TSLogs << "success!\n";

    gLoRaPingNodes.start(); // start node ping/keep alive timer
    fflush(stdout);
}

int32_t GetSecPastRXWindow(){
    int32_t after_sec = (int32_t)GetHHWakeElapsed() - HOLYHOP_RX_WINDOW_SEC;
    return after_sec;

    //todo or if we are about to RX/TX outside the RX Window, dont post cloud data
}

void ProcessLoRa(){
    /////////////////////
    // print some debugging info
    //
    static bool fshow_interval = false;
    if ( !fshow_interval && GetHHWakeElapsed() <= HOLYHOP_RX_WINDOW_SEC ){
        fshow_interval = true;
        TSLogs.Time() << "******Sensor RX window OPEN******\n";
        // save network state to disk
        TSLogs << "DEBUG saving network state..\n";
        if ( !SaveNetworkState() ){
            TSLogs.Time() << "ERROR failed to save network state\n";
        }
    }
    static bool fshow_tlmwait = false;
    if ( fshow_interval && !fshow_tlmwait && GetHHWakeElapsed() > HOLYHOP_TX_EXTRA_WAIT_SEC ){
        fshow_tlmwait = true;
        TSLogs.Time() << "******Base TLM wait ended******\n";
    }
    if ( fshow_interval && GetHHWakeElapsed() > HOLYHOP_RX_WINDOW_SEC ){
        fshow_interval = false;
        fshow_tlmwait = false;
        TSLogs.Time() << "******Sensor RX window CLOSED******\n";
    }


    /////////////////////////
    // process webservice requests using condition variables locks & flags
    //
    // get latest node meta data
    if ( gfGetNodesMeta ){
        std::lock_guard lockit(gSharedDataMutex);
        gHHNodesMeta.clear();
        const std::vector<HHNodeInfo_t> &nodes = GetHHNodes();
        for(const auto node : nodes){
            std::string nname = GetHHNodeName(node.dev_id); // use name from here (includes <unknown>)
            std::string hhver = node.protocol_ver == 0 ? "unknown" : fmt::format("{:04X}", node.protocol_ver);
            std::string fwver = node.firmware_ver == 0 ? "unknown" : fmt::format("{:04X}", node.firmware_ver);
            std::string blver = node.bootloader_ver == 0 ? "unknown" : fmt::format("{:08X}", node.bootloader_ver);
            std::string prf_uplink = node.preferred_uplink_id.IsNull() ? "" : node.preferred_uplink_id;
            RadarConfiguration_t rcfg;
            PayloadRadarCfgToRadarCfg( node.radar_cfg,rcfg );

            HHNodeMeta_t node_meta = {
                node.dev_id.c_str(),
                hhver,
                fwver,
                blver,
                nname,
                node.last_tx_time,
                rcfg,
                node.wake_count,
                node.base_hops,
                node.fw_path,
                node.fw_offset,
                prf_uplink
            };

            for(const auto qcmd : node.cmd_queue){
                HHNodeMeta_t::HHNodeCmdMeta_t ncmd = {
                    GetHHPktTypeName(qcmd.pkt_type),
                    //GetHHPktReplyStatusName(qcmd.reply),
                    //std::string()
                };
                node_meta.cmdq.push_back(ncmd);
            }

            gHHNodesMeta.push_back(node_meta);
        }
        // signal done
        gfGetNodesMeta = false;
        gSharedDataCV.notify_one();
    }

    // if flagged recalculate all telemetry offsets
    if ( gfReorderTlmOffsets ){
        std::lock_guard lockit(gSharedDataMutex);
        ReorderHHNodeTlmOffsets();
        // signal done
        gfReorderTlmOffsets = false;
        gSharedDataCV.notify_one();
    }

    // if flagged restart the wake interval timer
    if ( gfRestartWakeInterval ){
        TSLogs.Time() << "DEBUG Restarting Wake Interval Timer\n";
        std::lock_guard lockit(gSharedDataMutex);
        SetHHWakeNext( 10, GetHHWakeInterval() ); // in 10 seconds
        // signal done
        gfRestartWakeInterval = false;
        gSharedDataCV.notify_one();
    }

    // if flagged manually add a node
    if ( gfManuallyAddNode ){
        std::lock_guard lockit(gSharedDataMutex);
        HHNodeInfo_t node = InitHHNodeInfo();
        node.dev_id = gManuallyAddNodeID;
        AddHHNode( node );
        // signal done
        gfManuallyAddNode = false;
        gSharedDataCV.notify_one();
    }

    //////////////////////////////////////////////
    // resync sensor nodes
    //
    // periodically resync node's we haven't heard for a while
    // todo.

    // resync all nodes
    if ( GetHHFlagResyncAll() ){
        ClearHHFlagResyncAll();
        const std::vector<HHNodeInfo_t> &nodes = GetHHNodes();
        for(const auto node : nodes){
            // if it doesn't already have a sync command queued.
            if ( node.cmd_queue.size() &&
                node.cmd_queue.front().pkt_type == PacketType_t::CMD_SET_INTERVAL ){
                TSLogs.Time() << fmt::format("{} already has CMD_SET_INTERVAL queued\n", node.dev_id);
            }
            // queue one
            else{
                bool fsend_now = false;
                CmdNodeToSync(node.dev_id, fsend_now);
            }
        }
    }

    ////////////////////////////////////////////
    // 'ping' sensor nodes periodically so they know they are still in sync
    // (It resets their 'forget' timer when they hear from us)
    //
    if ( gLoRaPingNodes.elapsed_sec() >= HOLYHOP_PING_NODES_SEC ){
        gLoRaPingNodes.restart();
        TSLogs.Time() << "### Time to ping nodes.\n";
        // use a get version packet. (any pkt type from us would work.)
        const std::vector<HHNodeInfo_t> &nodes = GetHHNodes();
        for(const auto node : nodes){
            QueryNodeInfo( node.dev_id );
        }
    }

    ////////////////////////////////////////////
    // process threadsafe outgoing command queue
    //
    HHQueuedCommand_t qcmd;
    while( gHHCmdQueue.try_pop(qcmd) ){
        //TSLogs.Time() << fmt::format("gHHCmdQueue pop for {}\n",qcmd.dest_id);
        // For some cmd types, get rid of queued old duplicate commands. only keep the newest.
        bool fremove_dupes;
        switch( qcmd.pkt_type ){
            case PacketType_t::CMD_BEGIN_DFU_UPLOAD:
            case PacketType_t::CMD_VERIFY_DFU_UPLOAD:
            case PacketType_t::CMD_SET_INTERVAL:
            case PacketType_t::CMD_GET_INTERVAL:
            case PacketType_t::CMD_CONFIG_LORA:
            case PacketType_t::CMD_CONFIG_RADAR:
            case PacketType_t::CMD_GET_RADAR_CONFIG:
            case PacketType_t::CMD_GET_NAME:
            case PacketType_t::CMD_GET_VERSION:
            case PacketType_t::CMD_SET_NAME:
            case PacketType_t::CMD_RESET:
                fremove_dupes = true;
                break;
            default:
                fremove_dupes = false;
        }

        // special handling for PacketType_t::CMD_DFU_UPLOAD
        if ( qcmd.pkt_type == PacketType_t::CMD_DFU_UPLOAD ){
            // fill in the packet with actual fw data chunk
            BuildDFUUploadCmd( qcmd );
        }

        QueueHHNodeCmd(qcmd,fremove_dupes);
/*
        if ( qcmd.fsend_now == true ){
            // Intented for CmdSetInterval which is timing sensitive, or debugging.
            // End any ongoing reply wait, so that this fsend_now command will (in theory)
            // ..be sent immediately. (if not this command then another fsend_now command
            // ..that happens to come first.)
            gLoRaWaitReply.stop();
            gLoRaSendBufLen = 0;
        }
*/
    }

    /////////////////////////////////////////////////
    // send next waiting command
    //
    // Are we close to, or inside, the RX Window? For long running command sequences
    // ..like firmware update we don't want to clobber the RX Window.
    bool fclear_of_rxw = (GetHHWakeNextMS()/1000 > HOLYHOP_PRE_RX_WINDOW_SEC) &&
                     (GetHHWakeElapsed() > HOLYHOP_TX_EXTRA_WAIT_SEC);

    // IF we aren't already sending, AND we aren't waiting for a cmd reply, AND the node
    // ..has time to respond, then send next queued command.
    if ( gLoRaSendBufLen == 0 && !gLoRaWaitReply.isRunning() && fclear_of_rxw){
        // process node command queues. find a command that needs to be sent now
        HHQueuedCommand_t acmd;
        if ( NextHHQCmd(acmd) ){
            // send command packet
            SendLoRaPacket(acmd.cmd_pkt_buf, acmd.pkt_size, acmd.dest_id);
        }
    }

    ////////////////////////////////////
    // do RX Reply wait timer and retry count.
    //
    if ( gLoRaWaitReply.elapsed_ms() >= HOLYHOP_REPLY_TIMEOUT ){
        gLoRaWaitReply.stop();
        TSLogs.Time() << "RX Reply Wait timed out.\n";
    }

    //////////////////////
    // process expired cmds
    RemoveHHQOldCmds();

    ///////////////////////////////
    // check for a LoRa radio event
    //
    if ( !gLoRaIrqFlag )
        return; // we are done here

    gLoRaIrqFlag = false;
    uint32_t irq_flags = radio.getIrqFlags();
    TSLogs.Time() << fmt::format("DEBUG irq flags 0x{:08X}\n", radio.getIrqFlags() );

    // handle radio interrupt events
    //
    // RX Done
    if ( irq_flags & RADIOLIB_SX126X_IRQ_RX_DONE ){
        uint8_t rx_buf[LORA_MAX_PKT_SIZE];
        size_t pkt_size = radio.getPacketLength();
        int rstate = radio.readData(rx_buf, sizeof(rx_buf));

        if ( rstate == RADIOLIB_ERR_RX_TIMEOUT ){
            TSLogs.Time() << fmt::format("RX TIMEOUT, code {}\n", rstate);
        }
        else if ( rstate == RADIOLIB_ERR_CRC_MISMATCH ){
            TSLogs.Time() << fmt::format("RX CRC error, code {}\n", rstate);
        }
        else if (rstate != RADIOLIB_ERR_NONE) {
            TSLogs.Time() << fmt::format("RX failed, code {}\n", rstate);
        }
        else{
            // valid LoRa packet
            float rssi = radio.getRSSI();
            float snr = radio.getSNR();
            float freq_off = radio.getFrequencyError();

            TSLogs.Time() << fmt::format("$$$$$ RX LoRa pkt {} bytes {: .2f} dBm rssi {: .2f} dB SNR {: .2f} Hz freq offset\n",
                pkt_size, rssi, snr, freq_off);
            // ("Raw pkt: ");
            // for ( size_t i=0; i < pkt_size; i++){
            //     ("%02X ", rx_buf[i]);
            // }
            // ("\n");

            // check for valid HH header
            if (pkt_size < sizeof(PacketHeader_t)){
                TSLogs.Time() << "RX invalid HHPacket header! ignored.\n";
            }
            else{
                // valid HH Packet. Process it.
                PacketHeader_t *ppkt_hdr = reinterpret_cast<PacketHeader_t *>(rx_buf);
                PacketDeviceIDStrHelper src_id_str(ppkt_hdr->Src);
                PacketDeviceIDStrHelper dest_id_str(ppkt_hdr->Dest);
                PacketDeviceIDStrHelper relayby_id_str(ppkt_hdr->RelayBy);
                PacketDeviceIDStrHelper relayto_id_str(ppkt_hdr->RelayTo);
                TSLogs.Time() << fmt::format("$$$$ HH packet from {} hops {} type: ({}) {}\n",
                    src_id_str, ppkt_hdr->HopCount, ppkt_hdr->PktType, GetHHPktTypeName(ppkt_hdr->PktType));
                TSLogs.Time() << fmt::format("$$$$ relayby {}, relayto {}, src {}, dest {}\n",
                                             relayby_id_str, relayto_id_str, src_id_str, dest_id_str );

                if (!ProcessHHPkt(ppkt_hdr, pkt_size, rssi, snr)) {
                    TSLogs.Time() << "ProcessHHPkt failed.\n";
                }
            }
        }
    }

    // TX Done
    if ( irq_flags & RADIOLIB_SX126X_IRQ_TX_DONE ){
        TSLogs.Time() << "TX done\n";
        radio.finishTransmit();
        // chk for send failure? timeout?
        gLoRaSendBufLen = 0;        // mark packet sent.
        gLoRaWaitReply.start();     // start waiting to RX the commmand reply

        // we sent a packet, back to listening
        int rstate = radio.startReceive();
        TSLogs.Time() << "[SX1262] Listening...";
        if (rstate != RADIOLIB_ERR_NONE) {
            TSLogs << fmt::format("failed, code {}\n", rstate);
            exit(-1);
            return;
        }
        TSLogs << "success.\n";
    }

    // something TIMEOUT
    if ( irq_flags & RADIOLIB_SX126X_IRQ_TIMEOUT ){
        TSLogs.Time() << fmt::format("TIMEOUT {:08X}\n", irq_flags);
    }

    // CAD done
    if ( irq_flags & RADIOLIB_SX126X_IRQ_CAD_DONE ){
        //RADIOLIB_SX126X_IRQ_CAD_DETECTED
    }
}
/*
template<class TPacket>
void SendLoRaPacket(TPacket &pkt, PacketDeviceID_t &dst_id) {
    static_assert(sizeof(pkt) <= LORA_MAXIMUM_PKT_SIZE);
    size_t pkt_size = sizeof(pkt);
    // set packet destination & source ids
    pkt.Header.Dest = dst_id;
    pkt.Header.Src = PacketDeviceID_t::GenericBaseID(); // from us

    PacketDeviceIDStrHelper dest_str(pkt.Header.Dest);
    PacketDeviceIDStrHelper src_str(pkt.Header.Src);
    ("Send LoRa packet to %s from us %s type %d size %d time on air %dms\n",
           dest_str.c_str(), src_str.c_str(), pkt.Header.PktType, pkt_size,
           radio.getTimeOnAir(pkt_size)/1000);

    // // do LoRa ChannelActivityDetection to see if channel is clear right now
    // StartLoRaCAD();
    // // when it ends we will send pkt (in the oncadtimeout handler)

    int rstate = radio.startTransmit(reinterpret_cast<uint8_t *>(&pkt), pkt_size);
    if (rstate != RADIOLIB_ERR_NONE){
        ("TX failed, code %d\n", rstate);
    }
}
*/

///////////////////////////////////////////////////////////////////////////////////
/// WEBSERVICE or other consumer THREAD
/// THREADSAFE functions
///

// force recalc all telemetry offsets
void RecalcTelemetryOffsets(){
    // signal main thread
    gfReorderTlmOffsets = true;
    //std::unique_lock lockit(gSharedDataMutex);
    //gSharedDataCV.wait(lockit,[]{ return gfReorderTlmOffsets==false; });
}

// get telemetry CSV column names description
std::string GetTelemetryColumns(){
    std::string tlm_columns = TELEMETRY_COLUMNS_RADARCFG;
    tlm_columns += "<br>";
    tlm_columns += TELEMETRY_COLUMNS_RADARTLM;
    return tlm_columns;
}

void GetWakeInterval( std::time_t &next_wake, uint32_t &wake_sec ){
    next_wake = GetHHWakeTime();
    wake_sec = GetHHWakeInterval();
}

const std::vector<HHNodeMeta_t> & GetHHNodesMetadata(){
    // signal main thread
    gfGetNodesMeta = true;
    std::unique_lock lockit(gSharedDataMutex);
    gSharedDataCV.wait(lockit,[]{ return gfGetNodesMeta==false; });
    return gHHNodesMeta;
}

bool GetLastRadarConfig( uint32_t dev_id_dword, RadarConfiguration_t &rcfg ){
    HHDeviceAddress_t dev_id(dev_id_dword);
    GetHHNodesMetadata(); // refresh metadata
    for ( auto nodemeta : gHHNodesMeta ){
        if ( nodemeta.dev_id == std::string(dev_id) ){
            rcfg = nodemeta.rad_cfg;
            return true;
        }
    }
    return false;
}

std::deque<std::string> read_last_lines( size_t nlines ) {
    std::deque<std::string> lines;
    // force log stream to flush and check file size.
    TSLogs << std::flush;
    std::filesystem::path log_path("base_server.log");

    std::error_code ec;
    if ( std::filesystem::file_size(log_path, ec) <= 0 ){
        return lines;
    }

    std::ifstream file(log_path, std::ios::ate); // open at end
    if (!file.is_open()) {
        TSLogs.Time() << "Error: Unable to open file " << log_path << " errno " << strerror(errno) << std::endl;
        return lines;
    }

    // Start reading backwards from the end
    std::string line;
    size_t newline_count = 0;
    size_t file_size = file.tellg();

    for (size_t pos = file_size - 1; pos >= 0 && newline_count < nlines; pos--) {
        file.seekg(pos);
        char c = file.get();

        if (c == '\n') {
            if (!line.empty()) {
                line.insert(0, 1, c);
                lines.push_front(std::move(line));
                line.clear();
                newline_count++;
            }
        } else {
            line.insert(0, 1, c);
        }

        if (pos == 0) {  // Beginning of file
            if (!line.empty()) {
                lines.push_front(std::move(line));
                newline_count++;
            }
            break;
        }
    }

    // If we didn't reach 200 lines, but we've read the whole file
    while (lines.size() > nlines) {
        lines.pop_front();
    }

    return lines;
}

std::string GetServerLog( size_t num_lines ){
    auto last_lines = read_last_lines( num_lines );
    std::string log_text;
    for (const auto& line : last_lines) {
        log_text += line;
    }
    return log_text;
}

// @brief Function to read a file and copy lines that contain the pattern
bool CopyLinesFromFile( const std::filesystem::path& input_filepath,
                        const std::filesystem::path& output_filepath,
                        const std::string &pattern_str) {
    // Open the input and output files for reading and writing respectively
    std::ifstream input_file(input_filepath);
    if (!input_file.is_open()) {
        TSLogs.Time() << "Error: Unable to open file " << input_filepath << " errno "
            << strerror(errno) << std::endl;
        return false;
    }

    // open output in APPEND mode
    std::ofstream output_file(output_filepath, std::ios_base::app);
    if (!output_file.is_open()) {
        TSLogs.Time() << "Error: Unable to create output file " << output_filepath << " errno "
            << strerror(errno) << std::endl;
        input_file.close();
        return false;
    }

    // Read the input file line by line
    std::string line_str;
    while (std::getline(input_file, line_str)) {
        if (!line_str.empty()) {  // Ignore empty lines
            // Check if the pattern is in the line & copy the line
            size_t pos = line_str.find(pattern_str);
            if (pos != std::string::npos) {
                output_file << line_str << std::endl;
            }
        }
    }

    input_file.close();
    output_file.close();

    // Check for errors
    if (input_file.is_open()) {
        TSLogs.Time() << "Error: Unable to close input file " << input_filepath << " errno "
            << strerror(errno) << std::endl;
    }
    if (output_file.is_open()) {
        TSLogs.Time() << "Error: Unable to close output file " << output_filepath << " errno "
            << strerror(errno) << std::endl;
    }

    return true; // success
}

// go thru all log files, picking out our sensor's data, put it in a temporary file
bool GetSensorLogFile( uint32_t dest_id_dword, std::filesystem::path &slog_filepath ){
    // create a temp file for the output
    std::stringstream sensor_id;
    sensor_id << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
        << dest_id_dword;
    TSLogs.Time() << fmt::format("GetSensorLogFile for {}\n", sensor_id.str());

    std::stringstream sensor_filename;
    sensor_filename << "sensor-" << sensor_id.str() << "-all.log";
    auto temp_filepath = std::filesystem::temp_directory_path() / sensor_filename.str();
    TSLogs.Time() << fmt::format("Using temp output file {}\n", temp_filepath.string());

    // delete any existing output file
    std::filesystem::remove( temp_filepath );

    // get a list of all .log files in the specified directory
    auto dir_path = std::filesystem::path(SENSOR_DATA_DIR);
    std::vector<std::filesystem::path> logfile_paths;
    try {
        for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".log") {  // regular files only, no dirs
                logfile_paths.push_back(entry.path().string());
            }
        }

        // Sort alphabetically using std::sort and std::lexicographical_order
        std::sort( logfile_paths.begin(), logfile_paths.end(),
                   [](const auto& lhs, const auto& rhs) {
                       return lhs < rhs;
                   }
        );

        // for each logfile
        for ( auto next_log_filepath : logfile_paths ) {
            //std::string next_log_filepath = "sensor-data-2025-04-02.log";
            if ( !CopyLinesFromFile( next_log_filepath, temp_filepath, sensor_id.str() ) )
                return false; // failure
        }
    } catch (const std::filesystem::filesystem_error& e) {
        // Handle any errors that occur while trying to access the directory
        TSLogs.Time() << "Error accessing directory: " << e.what() << '\n';
    }

    // did we actually get any results?
    std::error_code ec;
    if ( std::filesystem::file_size( temp_filepath, ec ) == 0 ){
        // no data, or non-existant sensor
        TSLogs.Time() << "No Data found for " << sensor_id.str() << " code " << ec << std::endl;
        return false;
    }

    // return the filepath of our output file in slog_filepath
    slog_filepath = temp_filepath;
    return true; // success
}

template<class TPacket>
void QueueHHCmd(TPacket &pkt, const HHDeviceAddress_t &dst_id, bool fsend_now, std::filesystem::path fw_path) {
    static_assert(sizeof(pkt) <= LORA_MAX_PKT_SIZE);

    size_t pkt_size = sizeof(pkt);
    pkt.Header.Dest = dst_id.ToDID();
    //moved to cmdpkt ctor: pkt.ref_id = rand() % 256;   // setting cmd ref_id. just a random # to match on.

    TSLogs.Time() << fmt::format("Queue LoRa packet to {} type ({}) {} size {}\n",
           dst_id, pkt.Header.PktType, GetHHPktTypeName(pkt.Header.PktType), pkt_size);

    // queue it
    HHQueuedCommand_t qitem = {};
    qitem.fsend_now = fsend_now;
    qitem.dest_id = dst_id;
    qitem.pkt_type = pkt.Header.PktType;
    qitem.ref_id = pkt.RefID;
    qitem.pkt_size = pkt_size;
    qitem.fw_path = fw_path;

    uint8_t *psrc = reinterpret_cast<uint8_t *>(&pkt);
    std::copy(psrc, psrc + pkt_size, qitem.cmd_pkt_buf.begin()); // copy pkt bytes

    gHHCmdQueue.push(qitem);
}

// overload that defaults to NOT fsend_now
template<class TPacket>
void QueueHHCmd(TPacket &pkt, const HHDeviceAddress_t &dst_id){
    QueueHHCmd(pkt, dst_id, false, {});
}

void QueueCmdRadarConfig( uint32_t dest_id_dword, RadarConfiguration_t &rcfg ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandConfigRadar_t pkt;
    pkt.Payload.start_dist_mm           = rcfg.start_dist_mm;
    pkt.Payload.end_dist_mm             = rcfg.end_dist_mm;
    pkt.Payload.max_profile             = rcfg.max_profile;
    pkt.Payload.peak_sorting            = rcfg.peak_sorting;
    pkt.Payload.threshold_method        = rcfg.threshold_method;
    pkt.Payload.threshold_sensitivity   = rcfg.threshold_sensitivity;
    pkt.Payload.fixed_amp_threshold     = rcfg.fixed_amp_threshold;
    pkt.Payload.fixed_str_threshold     = rcfg.fixed_str_threshold;
    pkt.Payload.signal_quality          = rcfg.signal_quality;
    pkt.Payload.max_step_count          = rcfg.max_step_count;
    pkt.Payload.reflector_shape         = rcfg.reflector_shape;
    pkt.Payload.close_range_leakage     = rcfg.close_range_leakage;
    QueueHHCmd(pkt, dest_id);
}

void QueueCmdGetRadarConfig( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandGetRadarConfig_t pkt;
    QueueHHCmd(pkt, dest_id);
}

void QueueCmdGetSensorInfo( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    QueryNodeInfo(dest_id);
}

void QueueCmdSetInterval( uint32_t tlm_interval_sec ){
    TSLogs.Time() << "######## setting wake interval to " << tlm_interval_sec << " seconds\n";
    SetHHWakeInterval( tlm_interval_sec );
    // signal main thread to resend all tlm offsets with new interval
    gfReorderTlmOffsets = true;
}

void QueueCmdSetSensorName( uint32_t dest_id_dword, std::string new_name ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandSetName_t pkt;
    new_name = new_name.substr( 0, HOLYHOP_NAME_SIZE-1 ); // length limited to HOLYHOP_NAME_SIZE-1
    pkt.Payload.name.Set( new_name.c_str(), new_name.length() );
    QueueHHCmd(pkt, dest_id);
}

void QueueCmdSetPreferredUplink( uint32_t dest_id_dword, uint32_t uplink_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    HHDeviceAddress_t uplink_id(uplink_id_dword);
    PacketCommandSetPreferredUplink_t pkt;
    pkt.Payload.uplink = uplink_id.ToDID();
    QueueHHCmd(pkt, dest_id);
}

void QueueCmdResetSensor( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandReset_t pkt;
    QueueHHCmd(pkt, dest_id);
}

// for debugging purposes, queue a command to immediately send a get version pkt.
// ..without waiting for wake time.
void QueueCmdSendPktNow( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandGetVersion_t pkt;
    QueueHHCmd(pkt, dest_id, true, {});
}

void QueueCmdBlinkLED( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandBlinkLED_t pkt;
    QueueHHCmd(pkt, dest_id);
}

// @brief apparently this algorithm is known as a CRC-16 CCITT FALSE
static uint16_t crc16_checksum(const uint8_t *data_p, uint32_t length) {
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return crc;
}

// @brief start firmware update process on a sensor
// @returns true on success
bool UpdateSensorFW( uint32_t dest_id_dword, const std::filesystem::path &fw_path ){
    HHDeviceAddress_t dest_id(dest_id_dword);

    // read firmware bin
    std::ifstream fw_file(fw_path, std::ios::binary);
    if (!fw_file.is_open()) {
        TSLogs.Time() << "Failed to open fw file: " << fw_path << "\n";
    }

    fw_file.seekg(0, std::ios_base::end);
    std::streamsize fw_size = fw_file.tellg();
    fw_file.seekg(0, std::ios::beg);

    std::vector<std::byte> fw_buffer;
    fw_buffer.resize(static_cast<std::size_t>(fw_size));
    if (!fw_file.read(reinterpret_cast<char*>(fw_buffer.data()), fw_size)){
        TSLogs.Time() << "Failed to read fw file: " << fw_path << "\n";
    }
    fw_file.close();

    // calc. checksum
    uint16_t crc16 = crc16_checksum( reinterpret_cast<uint8_t *>(fw_buffer.data()), fw_size);
    TSLogs.Time() << fmt::format("DEBUG fw_size {}, crc16 {:04X}\n", fw_size, crc16);

    // begin upload command
    PacketCommandBeginDFUUpload_t pkt;
    pkt.Payload.dfu_crc = crc16;
    pkt.Payload.dfu_size = fw_size;
    QueueHHCmd(pkt, dest_id, false, fw_path);

    return true;
}

// @brief resume stalled firmware update process on a sensor
// @returns true on success
bool ResumeSensorFW( uint32_t dest_id_dword ){
    HHDeviceAddress_t dest_id(dest_id_dword);
    PacketCommandDFUUpload_t pkt_dfu; // pkt will be built in queue handler
    bool fsend_now=false;   // send now false: queue cmd. once the cmd reply is received we do
                            // ..send now true to continously stream the fw chunks
    QueueHHCmd(pkt_dfu, dest_id, fsend_now, {});
    return true;
}

// @brief Add a node to the server, unless it already exists
bool ManuallyAddNode( uint32_t dev_id_dword ){
    // signal main thread and wait
    gManuallyAddNodeID.SetDW( dev_id_dword );
    gfManuallyAddNode = true;
    std::unique_lock lockit(gSharedDataMutex);
    gSharedDataCV.wait(lockit,[]{ return gfManuallyAddNode==false; });
    return true;
}

// @brief restart the wake interval timer NOW. does not change the interval.
void RestartWakeInterval(){
    // signal main thread
    gfRestartWakeInterval = true;
}

uint16_t GetHHVersion(){
    return HOLYHOP_VERSION;
}
