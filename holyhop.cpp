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

/// holyhop Base server and Sensor client code
///
/// Time crunch, resorted to using some global vars and mixing C++ paradigms with C. In other words: the usual!
/// todo: robust iterator/ptr invalidation checks
/// todo: see .h for more todo
///

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cassert>
#include "holyhop.h"

#define HHSENSOR_MAX_NODE_STORAGE 25

// some static global state
static HHDeviceAddress_t gHHLocalDID;   // our Device ID
static HHEvents_t gHHCallback = {};     // user callback pointers

#ifdef HOLYHOP_BASE
#include "timer.h"
static bool gfHHIsBase = true;        	// true if we are a Base, false if we are a sensor node
static bool gfHHResyncAll = false;      // true if all nodes need to be resyncd (we reordered tlm offsets)
static FDTimer gHHWakeTimer;            // server interval sync timer for telemetry/network comms
static std::vector<HHNodeInfo_t> gHHNodes(HOLYHOP_MAX_NODES); // todo: this sucks. use a list like deque maybe & clear old nodes
#else
static bool gfHHIsBase = false;         // true if we are a Base, false if we are a sensor node
#endif

#ifdef HOLYHOP_SENSOR
// uplink state data class
class HHUplinkInfo_t{
public:
    HHUplinkInfo_t() : BaseHopCount(0) {};
    
    void Set( const HHDeviceAddress_t &uplink, uint16_t bhop_count ) {
        UplinkDID = uplink;
        BaseHopCount = bhop_count;
    }

    void SetInvalid(){
        UplinkDID.SetNull();
        BaseHopCount = 0;
    }

    bool IsValid() const{
        return !UplinkDID.IsNull() && BaseHopCount != 0;
    }

    bool IsBase() const{
        return UplinkDID.IsBase();
    }

    // @returns true if uplink is valid and matches preferred uplink
    bool IsPreferred(){
        return IsValid() && UplinkDID == PrefUplinkDID;
    }

    HHDeviceAddress_t GetUplink() const{
        return UplinkDID;
    }

    uint16_t GetBaseHopCount() const{
        return BaseHopCount;
    }

    void SetPreferred( const HHDeviceAddress_t &pref_uplink ){
        PrefUplinkDID = pref_uplink;
    }

    HHDeviceAddress_t GetPreferred() const{
        return PrefUplinkDID;
    }

private:
    HHDeviceAddress_t UplinkDID;        // node to use as our uplink to base (could be base itself)
    uint16_t BaseHopCount;              // how many hops we are from Base, direct = 1, 1 hop = 2
    HHDeviceAddress_t PrefUplinkDID;    // a preferred uplink to use if available
} gHHUplink;

// node info array class.
// we want to involve as little C++ STL as possible on the Sensors. keep it basic.
class HHNodeList {
  public:
    HHNodeList() {
        clear();
    }

    void Add(HHNodeInfo_t &ni) {
        // find open slot
        HHNodeInfo_t *poldn = &NodeArray[0];    // at same time find oldest node
        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            HHNodeInfo_t *pnode = &NodeArray[idx];
            if (pnode->dev_id.IsNull()) {
                *pnode = ni;
                return;
            } else if (pnode->last_tx_time < poldn->last_tx_time)
                poldn = pnode;
        }

        // no open slots. overwrite oldest
        *poldn = ni;
        TSLogs.Time() << "HH no empty node slots! Overwriting oldest.\n";
    }

    HHNodeInfo_t *Get(size_t idx) {
        if (idx >= HHSENSOR_MAX_NODE_STORAGE)
            return nullptr;
        HHNodeInfo_t &ni = NodeArray[idx];
        if (ni.dev_id.IsNull())
            return nullptr;
        return &ni;
    }

    HHNodeInfo_t *Find(const HHDeviceAddress_t &dev_id) {
        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            if (NodeArray[idx].dev_id.IsMatch(dev_id))
                return &NodeArray[idx];
        }
        return nullptr;  // not found
    }

    // @brief determine if node is usable as an uplink/route to base
    // @returns true if so
    bool IsGoodHopToBase( const HHDeviceAddress_t &dev_id ){
        int min_snr = -15;
        if ( dev_id.IsNull() )
            return false;   // node is invalid

        HHNodeInfo_t *pnode = Find( dev_id );
        if ( pnode == nullptr )
            return false;   // node is unknown to us

        HHNodeInfo_t &node = *pnode;
        TSLogs << "DBG IsGoodHopToBase min_snr " << min_snr << " found node " << node.dev_id << " - last_snr " <<
            node.last_snr << ", islocal " << node.flocal << ", isbase " << node.dev_id.IsBase() <<
            ", base_hops " << node.base_hops << "\n";
        
        if ( node.last_snr < min_snr )
            return false;   // skip signal too weak

        if ( !node.flocal )
            return false;   // skip non-local node

        // if direct to Base, done searching! base is local / 1 hop.
        if (node.dev_id.IsBase())            
            return true;

        if (node.base_hops <= 0)
            return false;   // skip if the node is unsynchronized

        // dev_id is a good usable uplink/route to base!
        return true;
    }

    // @brief get local neighbor node w/ shortest hop that has snr >= min_snr
    // @returns true if route to base found, index of node to relay thru in bestn_idx
    bool FindShortestHopToBase(int8_t min_snr, size_t &bestn_idx){
        TSLogs.Time() << "DBG FindShortestHopToBase min_snr " << min_snr << "\n";
        // local nodes are any node that we saw as dev_id == relay_id
        // i.e. the node was its own relay origin
        uint16_t lowest_hops = std::numeric_limits<uint16_t>::max();  // searching for lowest hops
        bestn_idx = 0;

        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            HHNodeInfo_t &node = NodeArray[idx];
            if ( node.dev_id.IsNull() )
                continue;   // skip empty node 'slot'
            
            TSLogs << "DBG node " << node.dev_id << " - last_snr " << node.last_snr << ", islocal " <<
                node.flocal << ", isbase " << node.dev_id.IsBase() << ", base_hops " << node.base_hops << "\n";
            
            if ( node.last_snr < min_snr )
                continue;   // skip signal too weak nodes

            if ( !node.flocal )
                continue;   // skip non-local node

            // if we are direct to Base, done searching! base is local / 1 hop.
            if (node.dev_id.IsBase()){
                bestn_idx = idx;
                return true;
            }

            if (node.base_hops <= 0){
                continue;   // skip if the node is unsynchronized
            }

            // node is a shorter route then current best? use it
            if (node.base_hops < lowest_hops) {
                lowest_hops = node.base_hops;
                bestn_idx = idx;
            }
            // node is same distance as current best?
            else if (node.base_hops == lowest_hops && 
                     lowest_hops != std::numeric_limits<uint16_t>::max() ){
                // (we know bestn_idx is a valid index and a non-empty node slot at this point. follow logic above)
                // signal strength is the tie breaker.
                HHNodeInfo_t &bestn = NodeArray[bestn_idx];
                if ( node.last_snr > bestn.last_snr ) {
                    bestn_idx = idx;
                }
            }
        } // end for idx

        if ( lowest_hops == std::numeric_limits<uint16_t>::max()){
            TSLogs << "DBG didn't find any route to Base.\n";
            return false; // didnt find any route to Base
        }

        // found a route to base
        return true;
    }

    // @brief search local nodeinfo for shortest non-zero base hop count.
    // @returns How many hops base is from us using this best hop.
    uint16_t FindBestHopToBase(HHNodeInfo_t &bh_node) {
        bh_node = {};
        // find shortest route to base, but limit to stronger SNR if possible
        size_t bestn_idx = 0;
        if ( !FindShortestHopToBase( -1, bestn_idx ) ){
            if ( !FindShortestHopToBase( -128, bestn_idx ) ){
                return 0;
            }
        }

        // we found a route to base
        bh_node = NodeArray[bestn_idx];
        // direct to base? (1 hop)
        if ( bh_node.dev_id.IsBase() ){
            bh_node.base_hops = 1;
            return 1;
        }

        bh_node.base_hops += 1;     // add +1 hop for us
        return bh_node.base_hops;
    }

    // @returns true if we removed any nodes
    bool RemoveOldNodes(){
        std::time_t nowt = GetRAK4630Time();
        bool fremove_any = false;
        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            HHNodeInfo_t &node = NodeArray[idx];
            if ( node.dev_id.IsNull() )
                continue;   // skip empty node 'slot'

            if ( difftime(nowt,node.last_tx_time) >= HOLYHOP_FORGET_NODE_SEC ){
                std::time_t dtime = difftime(nowt,node.last_tx_time);
                TSLogs << "DBG removing node " << node.dev_id << " last_tx_time diff " << dtime << "\n";
                fremove_any = true;
                NodeArray[idx].dev_id.SetNull();
            }
        }
        return fremove_any;
    }

    // @brief system clock changed, fix datetimes
    void OnClockChange( const std::time_t time_offset ){
        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            HHNodeInfo_t &node = NodeArray[idx];
            if ( node.dev_id.IsNull() )
                continue;   // skip empty node 'slot'

            node.last_tx_time += time_offset;
        }
    }

    void clear() {
        for (size_t idx = 0; idx < HHSENSOR_MAX_NODE_STORAGE; idx++) {
            NodeArray[idx].dev_id.SetNull();
        }
    }

    size_t size() {
        return HHSENSOR_MAX_NODE_STORAGE;
    }

  private:
    // "slots" to store node data in. dev_id==null(0) means empty slot.
    HHNodeInfo_t NodeArray[HHSENSOR_MAX_NODE_STORAGE];
} gHHNodes;

#endif

const char *GetHHPktTypeName(PacketType_t pkt_type) {
    switch (pkt_type) {
        case PacketType_t::UNSET:
            return "UNSET";
        case PacketType_t::CMD_RESET:
            return "CMD_RESET";
        case PacketType_t::CMD_GET_VERSION:
            return "CMD_GET_VERSION";
        case PacketType_t::CMD_GET_NAME:
            return "CMD_GET_NAME";
        case PacketType_t::CMD_SET_NAME:
            return "CMD_SET_NAME";
        case PacketType_t::CMD_GET_INTERVAL:
            return "CMD_GET_INTERVAL";
        case PacketType_t::CMD_SET_INTERVAL:
            return "CMD_SET_INTERVAL";
        case PacketType_t::CMD_CONFIG_LORA:
            return "CMD_CONFIG_LORA";
        case PacketType_t::CMD_CONFIG_RADAR:
            return "CMD_CONFIG_RADAR";
        case PacketType_t::CMD_GET_RADAR_CONFIG:
            return "CMD_GET_RADAR_CONFIG";
        case PacketType_t::CMD_BEGIN_DFU_UPLOAD:
            return "CMD_BEGIN_DFU_UPLOAD";
        case PacketType_t::CMD_DFU_UPLOAD:
            return "CMD_DFU_UPLOAD";
        case PacketType_t::CMD_VERIFY_DFU_UPLOAD:
            return "CMD_VERIFY_DFU_UPLOAD";
        case PacketType_t::CMD_BLINK_LED:
            return "CMD_BLINK_LED";
        case PacketType_t::CMD_GET_PREFERRED_UPLINK:
            return "CMD_GET_PREFERRED_UPLINK";
        case PacketType_t::CMD_SET_PREFERRED_UPLINK:
            return "CMD_SET_PREFERRED_UPLINK";

        case PacketType_t::REPLY_VERSION:
            return "REPLY_VERSION";
        case PacketType_t::REPLY_SUCCESS:
            return "REPLY_SUCCESS";
        case PacketType_t::REPLY_FAILURE:
            return "REPLY_FAILURE";
        case PacketType_t::REPLY_NAME:
            return "REPLY_NAME";
        case PacketType_t::REPLY_INTERVAL:
            return "REPLY_INTERVAL";
        case PacketType_t::REPLY_RADAR_CONFIG:
            return "REPLY_RADAR_CONFIG";
        case PacketType_t::REPLY_DFU_UPLOAD:
            return "REPLY_DFU_UPLOAD";
        case PacketType_t::REPLY_VERIFY_DFU_UPLOAD:
            return "REPLY_VERIFY_DFU_UPLOAD";
        case PacketType_t::REPLY_PREFERRED_UPLINK:
            return "REPLY_PREFERRED_UPLINK";

        case PacketType_t::RADAR_TELEMETRY:
            return "RADAR_TELEMETRY";
        case PacketType_t::RADAR_GNSS_TELEMETRY:
            return "RADAR_GNSS_TELEMETRY";
		case PacketType_t::RADAR_ALLM_TELEMETRY:
			return "RADAR_ALLM_TELEMETRY";
        default:
            return "unknown pkt type";
    }
}

const char *GetHHPktReplyStatusName(PacketReplyStatus_t status) {
    switch (status) {
        case PacketReplyStatus_t::RSTATUS_UNSET:
            return "RSTATUS_UNSET";
        case PacketReplyStatus_t::RSTATUS_SUCCESS:
            return "RSTATUS_SUCCESS";
        case PacketReplyStatus_t::RSTATUS_FAILURE:
            return "RSTATUS_FAILURE";
        case PacketReplyStatus_t::RSTATUS_TIMEOUT:
            return "RSTATUS_TIMEOUT";
        default:
            return "unknown reply status";
    }
}

// @brief initialize holyhop
void InitHH(bool fbase, const HHDeviceAddress_t &local_dev_id, HHEvents_t &events) {
    gfHHIsBase = fbase;  	// are we base or sensor?
    gHHLocalDID = local_dev_id;
    gHHCallback = events;
    gHHNodes.clear();
#ifdef HOLYHOP_BASE
    gfHHResyncAll = false;
    if ( !gHHWakeTimer.Create() )
        TSLogs.Time() << "Create wake timer failed!\n";
    if ( !gHHWakeTimer.Start( 10, HOLYHOP_DEFAULT_TLM_INTERVAL_SEC ) )
        TSLogs.Time() << "Start wake timer failed!\n";
#else
    gHHUplink.SetInvalid();
#endif    
}

#ifdef HOLYHOP_BASE
// get time of next sensor wake period
std::time_t GetHHWakeTime(){
    std::time_t nowt = std::time(nullptr);
    nowt += gHHWakeTimer.GetSecondsLeft();
    return nowt;
}

// get seconds since current wake period started
uint32_t GetHHWakeElapsed(){
    return gHHWakeTimer.GetSeconds();
}

// get milliseconds until next sensor wake period
uint32_t GetHHWakeNextMS(){
    return gHHWakeTimer.GetMSLeft();
}

// get current telemetry/network comms wake interval
uint32_t GetHHWakeInterval(){
    return gHHWakeTimer.GetIntervalSeconds();
}

// change the wake interval and the time to next wake
void SetHHWakeNext( int32_t start_in_sec, uint32_t tlm_int_sec ){
    if ( !gHHWakeTimer.Start(start_in_sec, tlm_int_sec) ){
        TSLogs.Time() << "Timer Start failed in SetHHWakeNext!\n";
    }
}

// change the wake interval w/o interrupting next wake time
void SetHHWakeInterval( uint32_t tlm_int_sec){
    uint32_t start_in_sec = std::max( GetHHWakeNextMS()/1000, (uint32_t) 1 );
    if ( !gHHWakeTimer.Start(start_in_sec, tlm_int_sec) ){
        TSLogs.Time() << "Timer Start failed in SetHHWakeInterval!\n";
    }
}

const std::vector<HHNodeInfo_t> & GetHHNodes(){
    return gHHNodes;
}

HHNodeInfo_t * FindHHNodeInfo(const HHDeviceAddress_t &node_id){
    auto niter = std::find_if(
        gHHNodes.begin(),
        gHHNodes.end(),
        [&node_id](const HHNodeInfo_t &elem) {
            return elem.dev_id.IsMatch(node_id);
        }
    );
    if ( niter == gHHNodes.end() )
        return nullptr;
    HHNodeInfo_t *pnode = &(*niter);
    return pnode;
}

// A new node will be at the end of the node storage, making it last in the sequence.
uint32_t CalcTlmOffsetMS(int num_hops, uint32_t prev_offset_ms){
    uint32_t base_tlm_time = 5; // time it takes base to process a tlm pkt & be ready for next
    uint32_t tlm_pkt_toa = HOLYHOP_TLM_PKT_TIME; // time on air for tlm packet todo:calc. this
    uint32_t padding_ms = 150;
    uint32_t hop_time = tlm_pkt_toa + LORA_TURNAROUND_DELAYMS + LORA_CAD_DELAYMS;
    uint32_t this_offset_ms =
        prev_offset_ms + base_tlm_time + hop_time * num_hops + padding_ms;
    return this_offset_ms;
}

// @brief recalculate all node telemetry offsets.
void ReorderHHNodeTlmOffsets(){
    TSLogs.Time() << "DEBUG ReorderHHNodeTlmOffsets\n";
    gfHHResyncAll = true; // flag the resync
    uint32_t prev_offset_ms = 0;
    for ( auto &node : gHHNodes ){
        // we are using relay_hops rather than base_hops which might be unknown still
        node.tlm_offset_ms = prev_offset_ms;
        prev_offset_ms = CalcTlmOffsetMS(node.relay_hops, prev_offset_ms);
        TSLogs.Time() << "DEBUG node " << node.dev_id << " this_offset_ms " <<  node.tlm_offset_ms << "\n";
    }

    int32_t tlm_left_ms = HOLYHOP_TX_EXTRA_WAIT_SEC*1000 - prev_offset_ms;
    TSLogs.Time() << "DEBUG ReorderHHNodeTlmOffsets remaining telemetry window is "
        << tlm_left_ms << "ms\n";
}

// @brief Determine what a new node's telemetry offset time in ms will be.
// This helps us avoid TX collisions. When a sensor wakes on the wake interval it will wait
// this many milliseconds to send tlm.
uint32_t CalcHHNodeTlmOffsetMS(int num_hops){
    uint32_t prev_offset_ms = gHHNodes.size() ? gHHNodes.back().tlm_offset_ms : 0;
    uint32_t this_offset_ms = CalcTlmOffsetMS(num_hops, prev_offset_ms);
    TSLogs.Time() << "DEBUG CalcHHNodeTlmOffsetMS this_offset_ms " << this_offset_ms << "\n";

    // eventually as nodes join or drop out and rejoin we run out of time in the telemetry portion of
    // ..the RX window. if that happens we reorder the whole sequence from zero.
    if ( this_offset_ms/1000 >= HOLYHOP_TX_EXTRA_WAIT_SEC ){
        TSLogs.Time() << "DEBUG this_offset_ms is past the telemetry window!\n";
        ReorderHHNodeTlmOffsets();
        // recalc. it may still be outside the tlm window if there are too many nodes
        // ..that are too distant. We allow it - better to limp along than exit.
        prev_offset_ms = gHHNodes.size() ? gHHNodes.back().tlm_offset_ms : 0;
        this_offset_ms = CalcTlmOffsetMS(num_hops, prev_offset_ms);
        if ( this_offset_ms/1000 >= HOLYHOP_TX_EXTRA_WAIT_SEC ){
            TSLogs.Time() << "DEBUG WARNING this_offset_ms is STILL past the telemetry window!\n";
        }
    }

    return this_offset_ms;
}

// @brief so loracomms layer can check if it needs to resync all nodes (due to reordering tlm offsets)
bool GetHHFlagResyncAll(){
    return gfHHResyncAll;
}

// @brief so loracomms layer can clear the flag.
void ClearHHFlagResyncAll(){
    gfHHResyncAll = false;
}
#endif
#ifdef HOLYHOP_SENSOR
HHNodeInfo_t *FindHHNodeInfo(const HHDeviceAddress_t &node_id) {
    HHNodeInfo_t *pnode = gHHNodes.Find(node_id);
    return pnode;
}

void DebugDumpNodes(){
    // debug: list all known nodes
    TSLogs << "DBG !********** HH Node list **********!\n";
    for ( size_t i=0; i<gHHNodes.size(); i++){
        HHNodeInfo_t *pnode = gHHNodes.Get(i);
        if ( !pnode )
            continue; // empty "slot"

        TSLogs << "DBG Node " << pnode->dev_id << ", last_txtime " << pnode->last_tx_time << ", last_snr " 
            << pnode->last_snr << ", islocal " << pnode->flocal << ", isbase " << pnode->dev_id.IsBase() 
            << ", base_hops " << pnode->base_hops << ", relay_hops " << pnode->relay_hops << ", relay_by " << pnode->rev_relay_id << "\n";
    }
}
#endif

// @brief initialize a HHNodeInfo_t struct
HHNodeInfo_t InitHHNodeInfo(){
    HHNodeInfo_t node_info = {}; // zero out struct
#ifdef HOLYHOP_BASE    
    node_info.radar_cfg.start_dist_mm = -1; // invalid rcfg
    node_info.reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
    //node_info.preferred_uplink_id.SetNull();
#endif
    return node_info;
}

// @brief store node info for a node we heard talking. could be new or existing.
// @param src_id is the origin of the packet
// @param relay_id if the packet has been relayed to us, this is who relayed it
// @param hop_count packet hop count thus far
// @param last_rssi local rssi when packet received
// @param last_snr local snr when packet received
void UpdateHHNodeInfo(const HHDeviceAddress_t &src_id, const HHDeviceAddress_t &relayby_id,
                      const HHDeviceAddress_t &relayto_id,
                      uint8_t relay_hops, int16_t last_rssi, int8_t last_snr ){
    // see if node is known
    HHNodeInfo_t *pnode = FindHHNodeInfo(src_id);
    // not known? new node
    if (pnode == nullptr) {
        TSLogs.Time().Fmt("HH Heard New Node %s\n", src_id.c_str());
        HHNodeInfo_t node_info = InitHHNodeInfo();
        // init new node
        node_info.dev_id = src_id;
        node_info.flocal = relayby_id == src_id;      // if we heard node relay itself. it is local.
        node_info.last_rssi = last_rssi;
        node_info.last_snr = last_snr;
#ifdef HOLYHOP_BASE
        node_info.rev_relay_id = relayby_id;
        node_info.relay_hops = relay_hops;
        node_info.last_tx_time = std::time(nullptr);  // now
        node_info.tlm_offset_ms = CalcHHNodeTlmOffsetMS(relay_hops);

        gHHNodes.push_back(node_info); // add the node to array
#endif
#ifdef HOLYHOP_SENSOR
        if ( relayto_id == gHHLocalDID ){       // only update rev_relay_id if packet was sent to US
            node_info.rev_relay_id = relayby_id;
            node_info.relay_hops = relay_hops;
        }
        node_info.last_tx_time = GetRAK4630Time();  // now
        gHHNodes.Add(node_info);  // add the node to array
#endif
        // call user callback
        if (gHHCallback.OnNewNode != nullptr)
            gHHCallback.OnNewNode(src_id);
    }
    else {
        // update existing node info
        if ( relayby_id == src_id ) // if we ever hear node relay itself. it is local.
            pnode->flocal = true;            
        pnode->last_rssi = last_rssi;
        pnode->last_snr = last_snr;
#ifdef HOLYHOP_BASE
        pnode->last_tx_time = std::time(nullptr);  // now
        pnode->rev_relay_id = relayby_id;
        // if relay_hops INCREASES we need to recalc all node tlm offset timing,
        // because this node & every following node will need more time now.
        if ( relay_hops > pnode->relay_hops ){
            pnode->relay_hops = relay_hops;
            ReorderHHNodeTlmOffsets();
        }
#endif
#ifdef HOLYHOP_SENSOR
        pnode->last_tx_time = GetRAK4630Time();  // now        
        // only update relayby_id if relayto_id was us.        
        if ( relayto_id == gHHLocalDID ){
            pnode->rev_relay_id = relayby_id;
            pnode->relay_hops = relay_hops;
        }
#endif
    }
    //DebugDumpNodes();
}

#ifdef HOLYHOP_SENSOR
// @brief a system clock change occured, fixup datetimes
void UpdateHHDatetimes( const std::time_t time_offset ){
    gHHNodes.OnClockChange(time_offset);
}

// @brief maintain node & network status and state
// @returns false if we no longer have an uplink to Base
bool UpdateHHNetworkState(){
    TSLogs << "DBG UpdateHHNetworkState\n";
    // remove old nodes
    if ( gHHNodes.RemoveOldNodes() ){
        TSLogs << "DBG old nodes were removed\n";
        // nodes removed. is our uplink still valid?
        if ( gHHUplink.IsValid() && !FindHHNodeInfo(gHHUplink.GetUplink()) ){
            // uplink was removed. we have no uplink.
            TSLogs << "DBG Uplink was removed. No Uplink\n";
            gHHUplink.SetInvalid();
        }
    }

    // uplink still stable?
    if ( gHHUplink.IsValid() ){
        // find the nodeinfo, check if base_hops changed
        HHNodeInfo_t *pnode = FindHHNodeInfo(gHHUplink.GetUplink());
        if ( !pnode ){
            TSLogs << "ERR Uplink not found in NodeArray! " << gHHUplink.GetUplink() <<"\n";
            gHHUplink.SetInvalid();
        }
        else if (!gHHUplink.IsBase() && ((pnode->base_hops+1) != gHHUplink.GetBaseHopCount()) ){
            TSLogs << "DBG Uplink " << gHHUplink.GetUplink() << " base_hops was "
                << gHHUplink.GetBaseHopCount() << " now it is " << pnode->base_hops << "\n";
            gHHUplink.SetInvalid();
        }
    }

    DebugDumpNodes(); //debug
    TSLogs << "DBG Uplink " << gHHUplink.GetUplink() << " VALID flag: " << gHHUplink.IsValid() << "\n";
    return gHHUplink.IsValid();
}

bool IsHHUplink( const HHDeviceAddress_t &dev_id ){
    if ( !gHHUplink.IsValid() )
        return false;

    return gHHUplink.GetUplink() == dev_id;
}

bool IsHHUplinkValid(){
    return gHHUplink.IsValid();
}

// @brief Use node as our uplink
bool SetHHUplink(const HHDeviceAddress_t &dev_id){
    TSLogs << "DBG SetHHUplink " << dev_id << "\n";
/*
    // if preferred uplink is set and healthy/available use it
    if ( !gHHUplink.GetPreferred().IsNull() ){
        HHNodeInfo_t *pnode = FindHHNodeInfo(gHHUplink.GetPreferred());
        if ( pnode && pnode->base_hops != 0 ){
            gHHUplink.Set( pnode->dev_id, pnode->base_hops+1 );            
            TSLogs << "DBG Choosing preferred uplink " << gHHUplink.GetUplink() << " with " 
                << gHHUplink.GetBaseHopCount() << " hops.\n";
            return true;    // success
        }
    }
*/
    // see if passed in node is a valid choice for us to use as an uplink
    if ( dev_id.IsBase() ){
        gHHUplink.Set(dev_id, 1);
    }
    else{
        HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
        if ( !pnode )
            return false;

        if ( pnode->base_hops == 0 )
            return false;

        gHHUplink.Set(dev_id, pnode->base_hops+1);
    }

    TSLogs << "DBG using uplink " << gHHUplink.GetUplink() << " with " << gHHUplink.GetBaseHopCount() 
        << " hops.\n";
    return true; // success
}

#endif
#ifdef HOLYHOP_BASE
// @brief Manually add this node id to node array if not already -
void AddHHNode(const HHNodeInfo_t &node_info){
    // is node known to us? find it
    HHNodeInfo_t *pnode = FindHHNodeInfo(node_info.dev_id);
    if ( pnode ){
        TSLogs.Time() << fmt::format("DEBUG HH Add New Node {} Already Exists!!\n", node_info.dev_id);
        *pnode = node_info;
        return;
    }
    TSLogs.Time() << fmt::format("HH Add New Node {}\n", node_info.dev_id);
    gHHNodes.push_back(node_info); // add the node to array
    //UpdateHHNodeInfo(dev_id, dev_id, 0x00000000, 0, 0, 0, false);
    //pnode = FindHHNodeInfo(dev_id);
    //assert( pnode != nullptr );
}

// @brief add command to node's outgoing queue
// @pass bool frem_dupes true to remove duplicate already queued commands
void QueueHHNodeCmd(HHQueuedCommand_t &qcmd, bool frem_dupes){
    // is node known to us? find it
    HHNodeInfo_t *pnode = FindHHNodeInfo(qcmd.dest_id);
    if ( !pnode ){
        // a node we haven't heard from yet. new node.
        UpdateHHNodeInfo(qcmd.dest_id, qcmd.dest_id, 0x00000000, 0, 0, 0);
        pnode = FindHHNodeInfo(qcmd.dest_id);
        assert( pnode != nullptr );
    }

    // remove any existing duplicate commands
    if ( frem_dupes ){
        for ( auto icmd = pnode->cmd_queue.begin(); icmd != pnode->cmd_queue.end();){
            if ( icmd->pkt_type == qcmd.pkt_type ){
                icmd = pnode->cmd_queue.erase(icmd);
                TSLogs.Time() << fmt::format("$$ HH Removed duplicate qcmd node {} pkt_type ({}) {}\n",
                                             pnode->dev_id, qcmd.pkt_type, GetHHPktTypeName(qcmd.pkt_type));
            }
            else{
                ++icmd;
            }
        }
    }

    // limit cmdq size by removing oldest cmd
    while ( pnode->cmd_queue.size() > HOLYHOP_CMD_MAX_QCMDS ){
        TSLogs.Time() << fmt::format("HH MAX QCMDS reached. Remove Queued command for node {} name {}\n",
        pnode->dev_id, pnode->name );
        pnode->cmd_queue.pop_front();
    }

    // if fw_path is set, save it w/ node
    if ( !qcmd.fw_path.empty() ){
        pnode->fw_path = qcmd.fw_path;
        pnode->fw_offset = 0;
    }

    // if the fsend_now is true put the command at the front of the queue!
    if ( qcmd.fsend_now ){
        pnode->cmd_queue.push_front(qcmd);
        pnode->reply_status = RSTATUS_UNSET; // reset any ongoing reply wait
        pnode->reply_wait_count = 0;
    }
    // else put cmd at back of q
    else{
        pnode->cmd_queue.push_back(qcmd);
    }

    TSLogs.Time() << fmt::format("HH Queued command for node {} name {} fsend_now={} fw_name {} last_txtime_t {} cmdq size {}\n",
        pnode->dev_id, pnode->name, qcmd.fsend_now, qcmd.fw_path.filename().string(), pnode->last_tx_time, pnode->cmd_queue.size());
}
#endif

// @brief lookup name of a node
HHStrClass GetHHNodeName(const HHDeviceAddress_t &dev_id) {
    HHStrClass name;
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if (pnode && pnode->name.length() > 0)
        name = pnode->name;
    else
        name = "{unknown}";
    return name;
}

// @brief get tlm_offset_ms of a node
uint32_t GetHHNodeTlmOffsetMs(const HHDeviceAddress_t &dev_id){
    uint32_t toff_ms = 0;
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        toff_ms = pnode->tlm_offset_ms;
    return toff_ms;
}

// @brief set node's protocol versions
void SetHHNodeVersion(const HHDeviceAddress_t &dev_id, uint16_t hh_ver, uint16_t fw_ver, uint32_t bl_ver) {
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode ){
        pnode->protocol_ver = hh_ver;
        pnode->firmware_ver = fw_ver;
        pnode->bootloader_ver = bl_ver;
    }
}

// @brief set name of a node
void SetHHNodeName(const HHDeviceAddress_t &dev_id, const HHStrClass &node_name) {
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if (pnode) {
        pnode->name = node_name;
    }
}

// @brief set node's latest reported base hop count
void SetHHNodeBaseHopCount(const HHDeviceAddress_t &dev_id, uint16_t base_hops) {
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if (pnode) {
        pnode->base_hops = base_hops;
    }
}

/*
// @brief set node's Last TX time as now
void SetHHNodeLastTlmTime(const HHDeviceAddress_t &dev_id ){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode ){
        pnode->last_tx_time = std::time( nullptr ); // now!
    }
}
*/
#ifdef HOLYHOP_BASE
// @brief get preferred uplink of a node
HHDeviceAddress_t GetHHNodePreferredUplink( const HHDeviceAddress_t &dev_id ){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    HHDeviceAddress_t prf_uplink_id;
    if (pnode) {
        prf_uplink_id = pnode->preferred_uplink_id;
    }
    return prf_uplink_id;
}

// @brief set preferred uplink of a node
void SetHHNodePreferredUplink(const HHDeviceAddress_t &dev_id, const HHDeviceAddress_t &prf_uplink_id) {
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if (pnode) {
        pnode->preferred_uplink_id = prf_uplink_id;
    }
}

// @brief get wake_count of a node
uint32_t GetHHNodeWakeCount(const HHDeviceAddress_t &dev_id){
    uint32_t wakec = 0;
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        wakec = pnode->wake_count;
    return wakec;
}

// @brief get fw_offset of a node
uint32_t GetHHNodeFWOffset(const HHDeviceAddress_t &dev_id){
    uint32_t fw_offset = 0;
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        fw_offset = pnode->fw_offset;
    return fw_offset;
}

std::filesystem::path GetHHNodeFWPath(const HHDeviceAddress_t &dev_id){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        return pnode->fw_path;
    return {};
}

void SetHHNodeFWOffset(const HHDeviceAddress_t &dev_id, uint32_t fw_offset){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        pnode->fw_offset = fw_offset;
}

void ClearHHNodeFWPath(const HHDeviceAddress_t &dev_id ){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode )
        pnode->fw_path.clear();
}

// @brief store node's measuremnt interval
void SetHHNodeInterval(const HHDeviceAddress_t &dev_id, int32_t measurement_ms ){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode ){
        pnode->measurement_ms = measurement_ms;
    }
}

// @brief store node's last replyradarconfig payload
void SetHHNodeLastRadarCfg(const HHDeviceAddress_t &dev_id,
                           const PayloadReplyRadarConfig_t &radar_cfg ){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode ){
        pnode->radar_cfg = radar_cfg;
    }
}

// @brief store node's last reported wake_count
void SetHHNodeWakeCount(const HHDeviceAddress_t &dev_id, uint32_t wake_count){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( pnode ){
        pnode->wake_count = wake_count;
    }
}

bool IsRXWindowOpen(){
    // The sensor node RX window starts then the Wake Timer triggers and lasts for the wake interval.
    //
    // Compensate for HOLYHOP_TX_EXTRA_WAIT_SEC..
    // When nodes wake up they send telemetry, scheduled one after another.
    // Give that a chance to finish before we start sending commands etc..
    int32_t total_tlm_wait_time = HOLYHOP_TX_EXTRA_WAIT_SEC; // todo: calc this total from gHHNodes array
    int32_t wake_time_elapsed = gHHWakeTimer.GetSeconds();
    if ( wake_time_elapsed > total_tlm_wait_time && wake_time_elapsed <= HOLYHOP_RX_WINDOW_SEC ){
        //TSLogs.Time() << "**** Sensor RXWindow is open! **** TotalTlmWait was " << total_tlm_wait_time << "s\n";
        return true;
    }

    return false;
}

// limit historyq size
void LimitHHHistQSize( HHNodeInfo_t &node ){
    while ( node.cmd_hist.size() > HOLYHOP_CMD_MAX_QCMDS ){
        TSLogs.Time() << fmt::format("HH MAX History QCMDS reached. Remove history command for node {}\n",
            node.dev_id );
        node.cmd_hist.pop_front();
    }
}

// @brief Look at all nodes and all queued commands. Which one needs to send soonest (if any)?
//        This command will be sent next.
bool NextHHQCmd(HHQueuedCommand_t &qnowcmd){
    //std::time_t nowt = std::time(nullptr);
    // we are searching. this variable tracks what we find to be the best node.
    HHNodeInfo_t *pqnow_node = nullptr;

    // check all nodes
    for ( auto &node : gHHNodes ){
        // skip nodes with empty command queue
        if ( node.cmd_queue.empty() )
            continue;

        // is the dest rx window open or is the next command fsend_now?
        if ( IsRXWindowOpen() ||
             node.cmd_queue.front().fsend_now == true ){
            // send this node-command next
            pqnow_node = &node;
            break;
        }
    }

    // if we found a node w/ a command that needs to send now.
    // return a copy of the command, or remove it if too many retries
    if ( pqnow_node ){
        qnowcmd = pqnow_node->cmd_queue.front();//std::move(pqnow_node->cmd_queue.front());

        int max_retry_count = HOLYHOP_CMD_MAX_RETRY;
        // special case for CMD_RESET, we just wait for 1 try.
        // because if the sensor reset right away we wont get any reply and we don't want to repeat a reboot command.
        if ( qnowcmd.pkt_type == PacketType_t::CMD_RESET ){
            max_retry_count = 1;
        }

        // too many retries? move it to hist
        if ( pqnow_node->reply_wait_count >= max_retry_count ){
            TSLogs.Time() << fmt::format("HH Command MAX_RETRY {} qcmd pkt_type ({}) {}\n",
                pqnow_node->dev_id, qnowcmd.pkt_type, GetHHPktTypeName(qnowcmd.pkt_type));
            // move qcmd to history
            pqnow_node->cmd_queue.pop_front();
            pqnow_node->cmd_hist.push_back(qnowcmd);
            LimitHHHistQSize( *pqnow_node );
            // reset node wait count
            pqnow_node->reply_wait_count = 0;

            if ( gHHCallback.OnReplyTimeout )
                gHHCallback.OnReplyTimeout(pqnow_node->dev_id);

            return false; // command timed out, removed it
        }
        else if (pqnow_node->reply_wait_count > 0) {
            TSLogs.Time() << fmt::format("HH Command RETRY {} #{} qcmd pkt_type ({}) {}\n",
                pqnow_node->dev_id, pqnow_node->reply_wait_count,
                qnowcmd.pkt_type, GetHHPktTypeName(qnowcmd.pkt_type));
        }
        ++pqnow_node->reply_wait_count;
        pqnow_node->reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
        return true;
    }
    // no queued commands ready
    return false;
}

// @brief remove the node's 'active'/current command even if we havent waited for a reply
void IncrementHHCmdQ(const HHDeviceAddress_t &dev_id){
    HHNodeInfo_t *pnode = FindHHNodeInfo(dev_id);
    if ( !pnode )
        return; // unknown node, do nothing

    // move qcmd to history
    auto qcmd = pnode->cmd_queue.front();
    pnode->cmd_queue.pop_front();
    pnode->cmd_hist.push_back(qcmd);
    LimitHHHistQSize( *pnode );

    // reset reply/retry status
    pnode->reply_wait_count = 0;
    pnode->reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
}

// @brief check node's cmd_queue for expired commands
void RemoveHHQOldCmds(){
    std::time_t nowt = std::time(nullptr);
    // check all nodes
    for ( auto &node : gHHNodes ){
        // any commands queued?
        if ( node.cmd_queue.size() > 0 ){
            int32_t since_last_txtime = difftime(nowt,node.last_tx_time);
            if ( since_last_txtime > HOLYHOP_CMD_EXPIRE_SEC ){
                // remove one command from queue
                HHQueuedCommand_t qcmd = node.cmd_queue.front();
                node.cmd_queue.pop_front();
                TSLogs.Time() << fmt::format("HH Removed expired qcmd node {} pkt_type {}\n", node.dev_id, qcmd.pkt_type);
            }
        }
        // limit histq size also
        LimitHHHistQSize(node);
    }
}

// @brief See if we know how to relay a packet to this destination.
// @returns true if we can and sets relay did in relay_to
bool GetHHNextHopFor(const PacketDeviceID_t &dest_id, HHDeviceAddress_t &relay_to) {
    HHDeviceAddress_t dest_addr( dest_id );
    // is destination a base!?!
    if (dest_id.IsBase()) {
        TSLogs << "No relay found to base " << dest_addr << ", we are BASE!\n";
        return false;
    }
    // else see if we have info on this destination node.
    else {
        HHNodeInfo_t *pnode = FindHHNodeInfo(dest_addr);  // gHHNodes.Find(dest_addr);
        if ( pnode != nullptr ){
            // how did node's traffic get to us? send it back from whence it came!
            relay_to = pnode->rev_relay_id;
            TSLogs << "Have relay to " << dest_addr << " " << pnode->relay_hops << " hops away\n";
        }
        else{
            TSLogs << "No relay to " << dest_addr << "\n";
            return false;
        }
    }

    return true; // success
}
#endif

#ifdef HOLYHOP_SENSOR
// @brief See if we know how to relay a packet to this destination.
// @returns true if we can and sets relay did in relay_to
bool GetHHNextHopFor(const PacketDeviceID_t &dest_id, HHDeviceAddress_t &relay_to) {
    HHDeviceAddress_t dest_addr( dest_id );
    // is destination a base?
    if (dest_id.IsBase()) {
        // relay via uplink if set
        if ( gHHUplink.IsValid() ){
            relay_to = gHHUplink.GetUplink();
            TSLogs << "Have route to Base " << dest_addr << " via uplink " << relay_to << " hop count " 
                << gHHUplink.GetBaseHopCount() << "\n";
            return true; // success
        }
        // no uplink to base
        else{
            TSLogs << "No uplink to base " << dest_addr << ".\n";
            // is there a preferred uplink? and is it usable?
            if ( gHHNodes.IsGoodHopToBase( gHHUplink.GetPreferred() ) ){
                relay_to = gHHUplink.GetPreferred();
                TSLogs << "DBG Trying preferred uplink " << relay_to << " to base\n";
                return true; // success
            }
            // no, find best hop if any
            HHNodeInfo_t best_hop;
            auto bhop_count = gHHNodes.FindBestHopToBase(best_hop);
            if ( bhop_count > 0 ){
                relay_to = best_hop.dev_id;
                TSLogs << "DBG Trying best hop " << relay_to << " to base.\n";
                return true;
            }
            return false;
        }
    }
    // else see if we have info on this destination node.
    else {
        HHNodeInfo_t *pnode = gHHNodes.Find(dest_addr);
        if ( pnode != nullptr ){
            // how did node's traffic get to us? send it back from whence it came!
            relay_to = pnode->rev_relay_id;
            TSLogs << "Have route to " << dest_addr << " via " << relay_to << " hop count " 
                << pnode->relay_hops << "\n";
            return true; // success
        }
        else{
            TSLogs << "No route to " << dest_addr << "\n";
            return false;
        }
    }
}

// @brief how many hops from base are we
// @returns 0 if no uplink
uint16_t GetHHBaseHops() {
    return gHHUplink.IsValid() ? gHHUplink.GetBaseHopCount() : 0;
}

// our uplink if any
HHDeviceAddress_t GetHHUplinkAddr(){
    return gHHUplink.GetUplink();
}

// preferred uplink if any
HHDeviceAddress_t GetHHPreferredUplinkAddr(){
    return gHHUplink.GetPreferred();
}

#endif

/*
// @brief Determine if this is a command Reply packet
bool IsHHReplyPkt(PacketType_t pkt_type) {
    switch (pkt_type) {
        case PacketType_t::REPLY_VERSION:
        case PacketType_t::REPLY_SUCCESS:
        case PacketType_t::REPLY_FAILURE:
        case PacketType_t::REPLY_NAME:
        case PacketType_t::REPLY_INTERVAL:
        case PacketType_t::REPLY_RADAR_CONFIG:
        case PacketType_t::REPLY_DFU_UPLOAD:
        case PacketType_t::REPLY_VERIFY_DFU_UPLOAD:
        case PacketType_t::REPLY_PREFERRED_UPLINK:
            return true;
    }
    return false;
}
*/

// @brief is the packet to us directly or is it a broadcast packettemplate<class TPacket>
bool IsHHPktToUs( const PacketHeader_t &pkt_hdr ){
    bool fthis_dest = gHHLocalDID.IsMatch(pkt_hdr.RelayTo) && gHHLocalDID.IsMatch(pkt_hdr.Dest);
    return fthis_dest || pkt_hdr.Dest.IsBroadcast();
}

#ifdef HOLYHOP_BASE
// @brief special processing when we RX a cmd reply packet
template<class TPacket>
bool ProcessHHReplyPkt(TPacket *ppkt, uint16_t size){
    // is the node known?
    HHNodeInfo_t *pnode = FindHHNodeInfo(ppkt->Header.Src);
    if ( !pnode ){ // not known? do nothing
        TSLogs.Time() << fmt::format("HH Command Reply from unknown node {} ignored.\n",
                                     pnode->dev_id);
        return false;
    }
    // does the reply match our 'active' command? The 'active' command is the front() of the cmd_queue.
    if ( pnode->cmd_queue.empty() ){
        TSLogs.Time() << fmt::format("HH Command Reply but cmd_queue is empty for node {}.\n",
                                     pnode->dev_id);
        return false;
    }
    HHQueuedCommand_t qcmd = pnode->cmd_queue.front();
    if ( ppkt->RefID != qcmd.ref_id ){
        TSLogs.Time() << fmt::format("HH Command Reply ref_id didn't match for node {}. pkt {} qcmd {}\n",
            pnode->dev_id, ppkt->RefID, qcmd.ref_id);
        return false;
    }

    // set reply status
    PacketType_t reply_type = ppkt->Header.PktType;
    PacketReplyStatus_t reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
    int error_code = 0; // todo: error code constants
    if ( reply_type == PacketType_t::REPLY_FAILURE ){
        reply_status = PacketReplyStatus_t::RSTATUS_FAILURE;
    }
    else if ( reply_type == PacketType_t::REPLY_DFU_UPLOAD ){
        PacketReplyDFUUpload_t *prpkt = reinterpret_cast<PacketReplyDFUUpload_t *>(ppkt);
        reply_status = prpkt->Payload.fsuccess ?
            PacketReplyStatus_t::RSTATUS_SUCCESS : PacketReplyStatus_t::RSTATUS_FAILURE;
    }
    else if ( reply_type == PacketType_t::REPLY_VERIFY_DFU_UPLOAD ){
        PacketReplyVerifyDFUUpload_t *prpkt = reinterpret_cast<PacketReplyVerifyDFUUpload_t *>(ppkt);
        reply_status = prpkt->Payload.fsuccess ?
            PacketReplyStatus_t::RSTATUS_SUCCESS : PacketReplyStatus_t::RSTATUS_FAILURE;
    }
    else{
        reply_status = PacketReplyStatus_t::RSTATUS_SUCCESS;
    }

    // valid reply (no timeout) to active command. move qcmd to cmd_hist
    pnode->cmd_queue.pop_front();
    pnode->cmd_hist.push_back(qcmd);

    // reset node wait count
    pnode->reply_wait_count = 0;

    std::string rstatus_str = GetHHPktReplyStatusName(reply_status);
    TSLogs.Time() << fmt::format("HH Command Reply {} {} err {}\n",
        pnode->dev_id, rstatus_str, error_code);

    // call user callback
    if ( gHHCallback.OnReplyCmd )
        gHHCallback.OnReplyCmd(pnode->dev_id, reply_type, reply_status);

    return true;
}
#endif

// @brief validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckHHPkt(TPacket *ppkt, uint16_t size, void (*OnPacket)(TPacket *)) {
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "HH invalid packet size\n";
        return false;
    }

    if ( !IsHHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "HH Packet not for us. ignored.\n";
        return false;
    }

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}

#ifdef HOLYHOP_SENSOR
// @brief For Telemetry packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckHHTelemetryPkt(TPacket *ppkt, uint16_t size, void (*OnPacket)(TPacket *)) {
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "HH invalid telemetry packet size\n";
        return false;
    }
    // we accept all Telemetry packets even if not for us.
    // we use our neighbors wake/telemetry packet to learn base hops and interval.
    //
    // special handling for Telemetry BEFORE OnPacket
    // update base hop then call generic Telemetry user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeBaseHopCount(dev_id, ppkt->BaseHops);        
    TSLogs << "Sniffed base_hops for " << dev_id << " hop count " << ppkt->BaseHops << "\n";
    
    if ( gHHCallback.OnTelemetry )
        gHHCallback.OnTelemetry( dev_id, ppkt->BaseHops );

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}
#endif

#ifdef HOLYHOP_BASE
// @brief For Telemetry packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckHHTelemetryPkt(TPacket *ppkt, uint16_t size, void (*OnPacket)(TPacket *)) {
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "HH invalid telemetry packet size\n";
        return false;
    }

    if ( !IsHHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "HH Telemetry Packet not for us. ignored.\n";
        return false;
    }

    // special handling for Telemetry BEFORE OnPacket
    // update wake count and base hop then call generic Telemetry user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeBaseHopCount(dev_id, ppkt->BaseHops);
    SetHHNodeWakeCount(dev_id, ppkt->Payload.wake_count);
    
    if ( gHHCallback.OnTelemetry )
        gHHCallback.OnTelemetry( dev_id, ppkt->BaseHops );

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}

// @brief For Reply packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckHHReplyPkt(TPacket *ppkt, uint16_t size, void (*OnPacket)(TPacket *)) {
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "HH invalid reply packet size\n";
        return false;
    }

    if ( !IsHHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "HH Reply Packet not for us. ignored.\n";
        return false;
    }

    // special handling for replies BEFORE OnPacket
    if ( !ProcessHHReplyPkt( ppkt, size ) ){
        TSLogs << "ProcessHHReplyPkt failed!\n";
        return false;
    }

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}

void InterceptOnReplyVersion(PacketReplyVersion_t *ppkt){
    // save version then do user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeVersion(dev_id, ppkt->Payload.hh_version, ppkt->Payload.fw_version, ppkt->Payload.bl_version );

    if ( gHHCallback.OnReplyVersion )
        gHHCallback.OnReplyVersion(ppkt);
}

void InterceptOnReplyName(PacketReplyName_t *ppkt){
    // save name then do user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeName(dev_id, ppkt->Payload.name.Get());

    if ( gHHCallback.OnReplyName )
        gHHCallback.OnReplyName(ppkt);
}

void InterceptOnReplyPreferredUplink(PacketReplyPreferredUplink_t *ppkt){
    // save uplink then do user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    HHDeviceAddress_t prf_uplink_id(ppkt->Payload.uplink);
    SetHHNodePreferredUplink(dev_id, prf_uplink_id);

    if ( gHHCallback.OnReplyPreferredUplink )
        gHHCallback.OnReplyPreferredUplink(ppkt);
}

void InterceptOnReplyInterval(PacketReplyInterval_t *ppkt){
    // save interval then do user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeInterval(dev_id, ppkt->Payload.measurement_ms);

    if ( gHHCallback.OnReplyInterval )
        gHHCallback.OnReplyInterval(ppkt);
}

void InterceptOnReplyRadarConfig(PacketReplyRadarConfig_t *ppkt){
    // save rcfg then do user callback if any
    HHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetHHNodeLastRadarCfg(dev_id, ppkt->Payload);

    if ( gHHCallback.OnReplyRadarConfig )
        gHHCallback.OnReplyRadarConfig(ppkt);
}
#endif
#ifdef HOLYHOP_SENSOR
void InterceptOnCommandGetPreferredUplink(PacketCommandGetPreferredUplink_t *ppkt){
    //HHDeviceAddress_t dev_id(ppkt->Header.Src);
    TSLogs << "HH Get preferred uplink: " << gHHUplink.GetPreferred() << "\n";

    if ( gHHCallback.OnCommandGetPreferredUplink )
        gHHCallback.OnCommandGetPreferredUplink(ppkt);
}

void InterceptOnCommandSetPreferredUplink(PacketCommandSetPreferredUplink_t *ppkt){
    //HHDeviceAddress_t dev_id(ppkt->Header.Src);
    if ( !gHHLocalDID.IsMatch( ppkt->Payload.uplink ) ) // can't be self
        gHHUplink.SetPreferred( ppkt->Payload.uplink );
        
    TSLogs << "HH Set preferred uplink: " << gHHUplink.GetPreferred() << "\n";
    // Do the callback BEFORE changing current uplink.
    // So the reply to this command uses existing uplink.
    if ( gHHCallback.OnCommandSetPreferredUplink )
        gHHCallback.OnCommandSetPreferredUplink(ppkt);

    // NOW, if we have an uplink set that isn't the preferred uplink
    if ( !gHHUplink.IsPreferred() ){
        // unset the uplink to give it a chance to be chosen.
        gHHUplink.SetInvalid();
    }
}
#endif

// check some things and see if this packet can be rejected
bool IsHHPktValid(PacketHeader_t *ppkt_hdr, uint16_t size){
    // check for garbage addresses
    if ( !ppkt_hdr->Src.IsKnownOUI() ||
         !ppkt_hdr->Dest.IsKnownOUI() ||
         !ppkt_hdr->RelayBy.IsKnownOUI() ||
         !ppkt_hdr->RelayTo.IsKnownOUI() ){
        return false;
    }

    if ( ppkt_hdr->HopCount > HOLYHOP_MAX_HOP_COUNT ){
        TSLogs << "HH Invalid Pkt: HopCount maxed out.\n";
        return false;   // this packet is going in circles or had gone too far!
    }

    if ( ppkt_hdr->Dest.IsMatch( ppkt_hdr->Src ) ){
        TSLogs << "HH Invalid Pkt: from self to self.\n";
        return false;   // from/to self
    }

    if ( ppkt_hdr->RelayTo.IsMatch(ppkt_hdr->Src) ){
        TSLogs << "HH Invalid Pkt: relaying to self.\n";
        return false;   // from self back to self
    }

    if ( ppkt_hdr->RelayBy.IsMatch(ppkt_hdr->Dest) ){
        TSLogs << "HH Invalid Pkt: relaying away from self.\n";
        return false;   // relay away from dest
    }

    if ( ppkt_hdr->Src.IsMatch(gHHLocalDID.ToDID()) ){
        TSLogs << "HH Invalid Pkt: we RXed our own packet.\n";
        return false;   // RX our own packet!
    }

    return true;
}

// @brief RX new packet, determine packet type and call processing function
// @return true on succeess
bool ProcessHHPkt(PacketHeader_t *ppkt_hdr, uint16_t size, int16_t this_rssi, int16_t this_snr) {
    // we have a chance to reject some garbage packets here..
    if ( !IsHHPktValid(ppkt_hdr,size) ){
        TSLogs << "RX Pkt failed validity check. Ignoring.\n";
        return false;
    }

    // is packet for us? are we a base..
    if (gfHHIsBase) {
        // Is packet for us or is it a broadcast?
        // We may hear distant relayed packets that are dest us, but relayed to another node.
        // This check blocks those.
        bool fbase_dest = ppkt_hdr->RelayTo.IsBase() && ppkt_hdr->Dest.IsBase();
        if (!fbase_dest && !ppkt_hdr->Dest.IsBroadcast()) {
            TSLogs.Time() << "HH RX HHPacket not for us. ignored.\n";
            return false;
        }

        // save the talking node's info. This is how we discover nodes trying to reach us directly.
        HHDeviceAddress_t src_id(ppkt_hdr->Src);
        HHDeviceAddress_t relayby_id(ppkt_hdr->RelayBy);
        HHDeviceAddress_t relayto_id(ppkt_hdr->RelayTo);
        UpdateHHNodeInfo(src_id, relayby_id, relayto_id, ppkt_hdr->HopCount, this_rssi, this_snr);
    }
    // else we are a sensor node
    else {
        // save the talking node's info. This is how we discover all neighboring nodes.
        HHDeviceAddress_t src_id(ppkt_hdr->Src);
        HHDeviceAddress_t relayby_id(ppkt_hdr->RelayBy);
        HHDeviceAddress_t relayto_id(ppkt_hdr->RelayTo);
        UpdateHHNodeInfo(src_id, relayby_id, relayto_id, ppkt_hdr->HopCount, this_rssi, this_snr);

#ifdef HOLYHOP_SENSOR
        // are we being asked to relay this packet? do it
        if ( !gHHLocalDID.IsMatch(ppkt_hdr->Dest) && gHHLocalDID.IsMatch(ppkt_hdr->RelayTo) ){
            // ..well not if we don't have a synchronized uplink yet. we dont want to contribute
            // to wrong packet relaying. Makes noise and drains battery.
            if ( !gHHUplink.IsValid() ){
                TSLogs.Time() << "DBG HH RX we can't relay yet.\n";
                return false;
            }            

            if ( gHHCallback.OnRelayPkt ){
                gHHCallback.OnRelayPkt( ppkt_hdr, size );
                return true; // done with this pkt
            }
            TSLogs << "ERR No user callback for relaying!\n";
            return false; // no user callback for relay is strange! fail.
        }
#endif
    }

    // check size & do callbacks
    uint8_t *payload = NULL;
    switch (ppkt_hdr->PktType) {
#ifdef HOLYHOP_SENSOR
        // Any relaying happened above. This packet was directly heard/RX.
        // Command packets
        case PacketType_t::CMD_RESET:
            return CheckHHPkt(reinterpret_cast<PacketCommandReset_t *>(ppkt_hdr), size, gHHCallback.OnCommandReset);
            break;

        case PacketType_t::CMD_GET_VERSION:
            return CheckHHPkt(reinterpret_cast<PacketCommandGetVersion_t *>(ppkt_hdr), size, gHHCallback.OnCommandGetVersion);
            break;

        case PacketType_t::CMD_GET_NAME:
            return CheckHHPkt(reinterpret_cast<PacketCommandGetName_t *>(ppkt_hdr), size, gHHCallback.OnCommandGetName);
            break;

        case PacketType_t::CMD_SET_NAME:
            return CheckHHPkt(reinterpret_cast<PacketCommandSetName_t *>(ppkt_hdr), size, gHHCallback.OnCommandSetName);
            break;

        case PacketType_t::CMD_GET_INTERVAL:
            return CheckHHPkt(reinterpret_cast<PacketCommandGetInterval_t *>(ppkt_hdr), size, gHHCallback.OnCommandGetInterval);
            break;

        case PacketType_t::CMD_SET_INTERVAL:
            return CheckHHPkt(reinterpret_cast<PacketCommandSetInterval_t *>(ppkt_hdr), size, gHHCallback.OnCommandSetInterval);
            break;

        case PacketType_t::CMD_CONFIG_LORA:
            return CheckHHPkt(reinterpret_cast<PacketCommandConfigLoRa_t *>(ppkt_hdr), size, gHHCallback.OnCommandConfigLoRa);
            break;

        case PacketType_t::CMD_CONFIG_RADAR:
            return CheckHHPkt(reinterpret_cast<PacketCommandConfigRadar_t *>(ppkt_hdr), size, gHHCallback.OnCommandConfigRadar);
            break;

        case PacketType_t::CMD_GET_RADAR_CONFIG:
            return CheckHHPkt(reinterpret_cast<PacketCommandGetRadarConfig_t *>(ppkt_hdr), size, gHHCallback.OnCommandGetRadarConfig);
            break;

        case PacketType_t::CMD_BEGIN_DFU_UPLOAD:
            return CheckHHPkt(reinterpret_cast<PacketCommandBeginDFUUpload_t *>(ppkt_hdr), size, gHHCallback.OnCommandBeginDFUUpload);
            break;

        case PacketType_t::CMD_DFU_UPLOAD:
            return CheckHHPkt(reinterpret_cast<PacketCommandDFUUpload_t *>(ppkt_hdr), size, gHHCallback.OnCommandDFUUpload);
            break;

        case PacketType_t::CMD_VERIFY_DFU_UPLOAD:
            return CheckHHPkt(reinterpret_cast<PacketCommandVerifyDFUUpload_t *>(ppkt_hdr), size, gHHCallback.OnCommandVerifyDFUUpload);
            break;

        case PacketType_t::CMD_BLINK_LED:
            return CheckHHPkt(reinterpret_cast<PacketCommandBlinkLED_t *>(ppkt_hdr), size, gHHCallback.OnCommandBlinkLED);
            break;

        case PacketType_t::CMD_GET_PREFERRED_UPLINK:
            return CheckHHPkt(reinterpret_cast<PacketCommandGetPreferredUplink_t *>(ppkt_hdr), size, InterceptOnCommandGetPreferredUplink);
            break;

        case PacketType_t::CMD_SET_PREFERRED_UPLINK:
            return CheckHHPkt(reinterpret_cast<PacketCommandSetPreferredUplink_t *>(ppkt_hdr), size, InterceptOnCommandSetPreferredUplink);
            break;
#endif
#ifdef HOLYHOP_BASE
        // reply packets
        case PacketType_t::REPLY_VERSION:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyVersion_t *>(ppkt_hdr), size, InterceptOnReplyVersion);
            break;

        case PacketType_t::REPLY_SUCCESS:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplySuccess_t *>(ppkt_hdr), size, gHHCallback.OnReplySuccess);
            break;

        case PacketType_t::REPLY_FAILURE:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyFailure_t *>(ppkt_hdr), size, gHHCallback.OnReplyFailure);
            break;

        case PacketType_t::REPLY_NAME:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyName_t *>(ppkt_hdr), size, InterceptOnReplyName);
            break;

        case PacketType_t::REPLY_PREFERRED_UPLINK:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyPreferredUplink_t *>(ppkt_hdr),
                                   size, InterceptOnReplyPreferredUplink);
            break;

        case PacketType_t::REPLY_INTERVAL:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyInterval_t *>(ppkt_hdr), size, InterceptOnReplyInterval);
            break;

        case PacketType_t::REPLY_RADAR_CONFIG:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyRadarConfig_t *>(ppkt_hdr), size, InterceptOnReplyRadarConfig);
            break;

        case PacketType_t::REPLY_DFU_UPLOAD:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyDFUUpload_t *>(ppkt_hdr), size, gHHCallback.OnReplyDFUUpload);
            break;

        case PacketType_t::REPLY_VERIFY_DFU_UPLOAD:
            return CheckHHReplyPkt(reinterpret_cast<PacketReplyVerifyDFUUpload_t *>(ppkt_hdr),
                                   size, gHHCallback.OnReplyVerifyDFUUpload);
#endif

        // telemetry packets
        case PacketType_t::RADAR_TELEMETRY:
            return CheckHHTelemetryPkt(reinterpret_cast<PacketRadarTelemetry_t *>(ppkt_hdr), size, gHHCallback.OnRadarTelemetry);
            break;

        case PacketType_t::RADAR_GNSS_TELEMETRY:
            return CheckHHTelemetryPkt(reinterpret_cast<PacketRadarGNSSTelemetry_t *>(ppkt_hdr), size, gHHCallback.OnRadarGNSSTelemetry);
            break;

        default:
            // unknown packet type
            return false;
    }

    return true;  // success
}
