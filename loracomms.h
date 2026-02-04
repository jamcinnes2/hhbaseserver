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

#include <string>
#include <filesystem>
#include "radarsensor.h"

static const char * BASE_FIRMWARE_VERSION_STR = "0.93";

struct HHNodeMeta_t{    // node metadata for web service consumption
    std::string dev_id;
    std::string protocol_ver;
    std::string firmware_ver;
    std::string bootloader_ver;
    std::string dev_name;
    std::time_t last_tx_time;
    RadarConfiguration_t rad_cfg;
    uint32_t wake_count;
    uint32_t base_hops;
    std::string fw_path;
    uint32_t fw_offset;
    std::string preferred_uplink_id;

    struct HHNodeCmdMeta_t{
        std::string cmd_type;
        //std::string reply_status;
        //std::string reply_data;
    };
    std::vector<HHNodeCmdMeta_t> cmdq;
};

void InitLoRa();
void ProcessLoRa();
int32_t GetSecPastRXWindow();

std::string CTimeToStr( std::time_t ct );
std::string RadarCfgToStr( const RadarConfiguration_t &rcfg );

// THREADSAFE
void GetWakeInterval( std::time_t &next_wake, uint32_t &wake_sec );
std::string GetTelemetryColumns();
const std::vector<HHNodeMeta_t> & GetHHNodesMetadata();
bool GetLastRadarConfig( uint32_t dev_id_dword, RadarConfiguration_t &rcfg );
std::string GetServerLog( size_t num_lines );
void RecalcTelemetryOffsets();
bool GetSensorLogFile( uint32_t dest_id_dword, std::filesystem::path &slog_filepath );
void QueueCmdRadarConfig( uint32_t dest_id_dword, RadarConfiguration_t &rcfg );
void QueueCmdGetRadarConfig( uint32_t dest_id_dword );
void QueueCmdGetSensorInfo( uint32_t dest_id_dword );
void QueueCmdSetInterval( uint32_t dest_id_dword );
void QueueCmdSetSensorName( uint32_t dest_id_dword, std::string new_name );
void QueueCmdSetPreferredUplink( uint32_t dest_id_dword, uint32_t uplink_id_dword );
void QueueCmdResetSensor( uint32_t dest_id_dword );
void QueueCmdSendPktNow( uint32_t dest_id_dword );
void QueueCmdBlinkLED( uint32_t dest_id_dword );
bool UpdateSensorFW( uint32_t dest_id_dword, const std::filesystem::path &fw_path );
bool ResumeSensorFW( uint32_t dest_id_dword );
bool ManuallyAddNode( uint32_t dev_id_dword );
void RestartWakeInterval();
uint16_t GetHHVersion();
