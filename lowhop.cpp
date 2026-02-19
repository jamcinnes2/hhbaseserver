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

/// lowhop Base server and Sensor client code
///
/// Time crunch, resorted to using some global vars and mixing C++ paradigms with C. In other words: the usual!
/// todo: robust iterator/ptr invalidation checks
/// todo: see .h for more todo
///

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cassert>
#include "lowhop.h"

#define LHSENSOR_MAX_NODE_STORAGE 25

// some static global state
static LHDeviceAddress_t gLHLocalDID;   // our Device ID
static LHEvents_t gLHCallback = {};     // user callback pointers

#ifdef LOWHOP_BASE
#include "timer.h"
static bool gfLHIsBase = true;        	// true if we are a Base, false if we are a sensor node
static bool gfLHResyncAll = false;      // true if all nodes need to be resyncd (we reordered tlm offsets)
static FDTimer gLHWakeTimer;            // server interval sync timer for telemetry/network comms
static std::vector<LHNodeInfo_t> gLHNodes(LOWHOP_MAX_NODES); // todo: this sucks. use a list like deque maybe & clear old nodes
#else
static bool gfLHIsBase = false;         // true if we are a Base, false if we are a sensor node
#endif

#ifdef LOWHOP_SENSOR
// uplink state data class
class LHUplinkInfo_t{
public:
    LHUplinkInfo_t() : BaseHopCount(0) {};
    
    void Set( const LHDeviceAddress_t &uplink, uint16_t bhop_count ) {
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

    LHDeviceAddress_t GetUplink() const{
        return UplinkDID;
    }

    uint16_t GetBaseHopCount() const{
        return BaseHopCount;
    }

    void SetPreferred( const LHDeviceAddress_t &pref_uplink ){
        PrefUplinkDID = pref_uplink;
    }

    LHDeviceAddress_t GetPreferred() const{
        return PrefUplinkDID;
    }

private:
    LHDeviceAddress_t UplinkDID;        // node to use as our uplink to base (could be base itself)
    uint16_t BaseHopCount;              // how many hops we are from Base, direct = 1, 1 hop = 2
    LHDeviceAddress_t PrefUplinkDID;    // a preferred uplink to use if available
} gLHUplink;

// node info array class.
// we want to involve as little C++ STL as possible on the Sensors. keep it basic.
class LHNodeList {
  public:
    LHNodeList() {
        clear();
    }

    void Add(LHNodeInfo_t &ni) {
        // find open slot
        LHNodeInfo_t *poldn = &NodeArray[0];    // at same time find oldest node
        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            LHNodeInfo_t *pnode = &NodeArray[idx];
            if (pnode->dev_id.IsNull()) {
                *pnode = ni;
                return;
            } else if (pnode->last_tx_time < poldn->last_tx_time)
                poldn = pnode;
        }

        // no open slots. overwrite oldest
        *poldn = ni;
        TSLogs.Time() << "LH no empty node slots! Overwriting oldest.\n";
    }

    LHNodeInfo_t *Get(size_t idx) {
        if (idx >= LHSENSOR_MAX_NODE_STORAGE)
            return nullptr;
        LHNodeInfo_t &ni = NodeArray[idx];
        if (ni.dev_id.IsNull())
            return nullptr;
        return &ni;
    }

    LHNodeInfo_t *Find(const LHDeviceAddress_t &dev_id) {
        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            if (NodeArray[idx].dev_id.IsMatch(dev_id))
                return &NodeArray[idx];
        }
        return nullptr;  // not found
    }

    // @brief determine if node is usable as an uplink/route to base
    // @returns true if so
    bool IsGoodHopToBase( const LHDeviceAddress_t &dev_id ){
        int min_snr = -15;
        if ( dev_id.IsNull() )
            return false;   // node is invalid

        LHNodeInfo_t *pnode = Find( dev_id );
        if ( pnode == nullptr )
            return false;   // node is unknown to us

        LHNodeInfo_t &node = *pnode;
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

        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            LHNodeInfo_t &node = NodeArray[idx];
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
                LHNodeInfo_t &bestn = NodeArray[bestn_idx];
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
    uint16_t FindBestHopToBase(LHNodeInfo_t &bh_node) {
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
        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            LHNodeInfo_t &node = NodeArray[idx];
            if ( node.dev_id.IsNull() )
                continue;   // skip empty node 'slot'

            if ( difftime(nowt,node.last_tx_time) >= LOWHOP_FORGET_NODE_SEC ){
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
        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            LHNodeInfo_t &node = NodeArray[idx];
            if ( node.dev_id.IsNull() )
                continue;   // skip empty node 'slot'

            node.last_tx_time += time_offset;
        }
    }

    void clear() {
        for (size_t idx = 0; idx < LHSENSOR_MAX_NODE_STORAGE; idx++) {
            NodeArray[idx].dev_id.SetNull();
        }
    }

    size_t size() {
        return LHSENSOR_MAX_NODE_STORAGE;
    }

  private:
    // "slots" to store node data in. dev_id==null(0) means empty slot.
    LHNodeInfo_t NodeArray[LHSENSOR_MAX_NODE_STORAGE];
} gLHNodes;

#endif

const char *GetLHPktTypeName(PacketType_t pkt_type) {
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

const char *GetLHPktReplyStatusName(PacketReplyStatus_t status) {
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

// @brief initialize lowhop
void InitLH(bool fbase, const LHDeviceAddress_t &local_dev_id, LHEvents_t &events) {
    gfLHIsBase = fbase;  	// are we base or sensor?
    gLHLocalDID = local_dev_id;
    gLHCallback = events;
    gLHNodes.clear();
#ifdef LOWHOP_BASE
    gfLHResyncAll = false;
    if ( !gLHWakeTimer.Create() )
        TSLogs.Time() << "Create wake timer failed!\n";
    if ( !gLHWakeTimer.Start( 10, LOWHOP_DEFAULT_TLM_INTERVAL_SEC ) )
        TSLogs.Time() << "Start wake timer failed!\n";
#else
    gLHUplink.SetInvalid();
#endif    
}

#ifdef LOWHOP_BASE
// get time of next sensor wake period
std::time_t GetLHWakeTime(){
    std::time_t nowt = std::time(nullptr);
    nowt += gLHWakeTimer.GetSecondsLeft();
    return nowt;
}

// get seconds since current wake period started
uint32_t GetLHWakeElapsed(){
    return gLHWakeTimer.GetSeconds();
}

// get milliseconds until next sensor wake period
uint32_t GetLHWakeNextMS(){
    return gLHWakeTimer.GetMSLeft();
}

// get current telemetry/network comms wake interval
uint32_t GetLHWakeInterval(){
    return gLHWakeTimer.GetIntervalSeconds();
}

// change the wake interval and the time to next wake
void SetLHWakeNext( int32_t start_in_sec, uint32_t tlm_int_sec ){
    if ( !gLHWakeTimer.Start(start_in_sec, tlm_int_sec) ){
        TSLogs.Time() << "Timer Start failed in SetLHWakeNext!\n";
    }
}

// change the wake interval w/o interrupting next wake time
void SetLHWakeInterval( uint32_t tlm_int_sec){
    uint32_t start_in_sec = std::max( GetLHWakeNextMS()/1000, (uint32_t) 1 );
    if ( !gLHWakeTimer.Start(start_in_sec, tlm_int_sec) ){
        TSLogs.Time() << "Timer Start failed in SetLHWakeInterval!\n";
    }
}

const std::vector<LHNodeInfo_t> & GetLHNodes(){
    return gLHNodes;
}

LHNodeInfo_t * FindLHNodeInfo(const LHDeviceAddress_t &node_id){
    auto niter = std::find_if(
        gLHNodes.begin(),
        gLHNodes.end(),
        [&node_id](const LHNodeInfo_t &elem) {
            return elem.dev_id.IsMatch(node_id);
        }
    );
    if ( niter == gLHNodes.end() )
        return nullptr;
    LHNodeInfo_t *pnode = &(*niter);
    return pnode;
}

// A new node will be at the end of the node storage, making it last in the sequence.
uint32_t CalcTlmOffsetMS(int num_hops, uint32_t prev_offset_ms){
    uint32_t base_tlm_time = 5; // time it takes base to process a tlm pkt & be ready for next
    uint32_t tlm_pkt_toa = LOWHOP_TLM_PKT_TIME; // time on air for tlm packet todo:calc. this
    uint32_t padding_ms = 150;
    uint32_t hop_time = tlm_pkt_toa + LORA_TURNAROUND_DELAYMS + LORA_CAD_DELAYMS;
    uint32_t this_offset_ms =
        prev_offset_ms + base_tlm_time + hop_time * num_hops + padding_ms;
    return this_offset_ms;
}

// @brief recalculate all node telemetry offsets.
void ReorderLHNodeTlmOffsets(){
    TSLogs.Time() << "DEBUG ReorderLHNodeTlmOffsets\n";
    gfLHResyncAll = true; // flag the resync
    uint32_t prev_offset_ms = 0;
    for ( auto &node : gLHNodes ){
        // we are using relay_hops rather than base_hops which might be unknown still
        node.tlm_offset_ms = prev_offset_ms;
        prev_offset_ms = CalcTlmOffsetMS(node.relay_hops, prev_offset_ms);
        TSLogs.Time() << "DEBUG node " << node.dev_id << " this_offset_ms " <<  node.tlm_offset_ms << "\n";
    }

    int32_t tlm_left_ms = LOWHOP_TX_EXTRA_WAIT_SEC*1000 - prev_offset_ms;
    TSLogs.Time() << "DEBUG ReorderLHNodeTlmOffsets remaining telemetry window is "
        << tlm_left_ms << "ms\n";
}

// @brief Determine what a new node's telemetry offset time in ms will be.
// This helps us avoid TX collisions. When a sensor wakes on the wake interval it will wait
// this many milliseconds to send tlm.
uint32_t CalcLHNodeTlmOffsetMS(int num_hops){
    uint32_t prev_offset_ms = gLHNodes.size() ? gLHNodes.back().tlm_offset_ms : 0;
    uint32_t this_offset_ms = CalcTlmOffsetMS(num_hops, prev_offset_ms);
    TSLogs.Time() << "DEBUG CalcLHNodeTlmOffsetMS this_offset_ms " << this_offset_ms << "\n";

    // eventually as nodes join or drop out and rejoin we run out of time in the telemetry portion of
    // ..the RX window. if that happens we reorder the whole sequence from zero.
    if ( this_offset_ms/1000 >= LOWHOP_TX_EXTRA_WAIT_SEC ){
        TSLogs.Time() << "DEBUG this_offset_ms is past the telemetry window!\n";
        ReorderLHNodeTlmOffsets();
        // recalc. it may still be outside the tlm window if there are too many nodes
        // ..that are too distant. We allow it - better to limp along than exit.
        prev_offset_ms = gLHNodes.size() ? gLHNodes.back().tlm_offset_ms : 0;
        this_offset_ms = CalcTlmOffsetMS(num_hops, prev_offset_ms);
        if ( this_offset_ms/1000 >= LOWHOP_TX_EXTRA_WAIT_SEC ){
            TSLogs.Time() << "DEBUG WARNING this_offset_ms is STILL past the telemetry window!\n";
        }
    }

    return this_offset_ms;
}

// @brief so loracomms layer can check if it needs to resync all nodes (due to reordering tlm offsets)
bool GetLHFlagResyncAll(){
    return gfLHResyncAll;
}

// @brief so loracomms layer can clear the flag.
void ClearLHFlagResyncAll(){
    gfLHResyncAll = false;
}
#endif
#ifdef LOWHOP_SENSOR
LHNodeInfo_t *FindLHNodeInfo(const LHDeviceAddress_t &node_id) {
    LHNodeInfo_t *pnode = gLHNodes.Find(node_id);
    return pnode;
}

void DebugDumpNodes(){
    // debug: list all known nodes
    TSLogs << "DBG !********** LH Node list **********!\n";
    for ( size_t i=0; i<gLHNodes.size(); i++){
        LHNodeInfo_t *pnode = gLHNodes.Get(i);
        if ( !pnode )
            continue; // empty "slot"

        TSLogs << "DBG Node " << pnode->dev_id << ", last_txtime " << pnode->last_tx_time << ", last_snr " 
            << pnode->last_snr << ", islocal " << pnode->flocal << ", isbase " << pnode->dev_id.IsBase() 
            << ", base_hops " << pnode->base_hops << ", relay_hops " << pnode->relay_hops << ", relay_by " << pnode->rev_relay_id << "\n";
    }
}
#endif

// @brief initialize a LHNodeInfo_t struct
LHNodeInfo_t InitLHNodeInfo(){
    LHNodeInfo_t node_info = {}; // zero out struct
#ifdef LOWHOP_BASE
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
void UpdateLHNodeInfo(const LHDeviceAddress_t &src_id, const LHDeviceAddress_t &relayby_id,
                      const LHDeviceAddress_t &relayto_id,
                      uint8_t relay_hops, int16_t last_rssi, int8_t last_snr ){
    // see if node is known
    LHNodeInfo_t *pnode = FindLHNodeInfo(src_id);
    // not known? new node
    if (pnode == nullptr) {
        TSLogs.Time().Fmt("LH Heard New Node %s\n", src_id.c_str());
        LHNodeInfo_t node_info = InitLHNodeInfo();
        // init new node
        node_info.dev_id = src_id;
        node_info.flocal = relayby_id == src_id;      // if we heard node relay itself. it is local.
        node_info.last_rssi = last_rssi;
        node_info.last_snr = last_snr;
#ifdef LOWHOP_BASE
        node_info.rev_relay_id = relayby_id;
        node_info.relay_hops = relay_hops;
        node_info.last_tx_time = std::time(nullptr);  // now
        node_info.tlm_offset_ms = CalcLHNodeTlmOffsetMS(relay_hops);

        gLHNodes.push_back(node_info); // add the node to array
#endif
#ifdef LOWHOP_SENSOR
        if ( relayto_id == gLHLocalDID ){       // only update rev_relay_id if packet was sent to US
            node_info.rev_relay_id = relayby_id;
            node_info.relay_hops = relay_hops;
        }
        node_info.last_tx_time = GetRAK4630Time();  // now
        gLHNodes.Add(node_info);  // add the node to array
#endif
        // call user callback
        if (gLHCallback.OnNewNode != nullptr)
            gLHCallback.OnNewNode(src_id);
    }
    else {
        // update existing node info
        if ( relayby_id == src_id ) // if we ever hear node relay itself. it is local.
            pnode->flocal = true;            
        pnode->last_rssi = last_rssi;
        pnode->last_snr = last_snr;
#ifdef LOWHOP_BASE
        pnode->last_tx_time = std::time(nullptr);  // now
        pnode->rev_relay_id = relayby_id;
        // if relay_hops INCREASES we need to recalc all node tlm offset timing,
        // because this node & every following node will need more time now.
        if ( relay_hops > pnode->relay_hops ){
            pnode->relay_hops = relay_hops;
            ReorderLHNodeTlmOffsets();
        }
#endif
#ifdef LOWHOP_SENSOR
        pnode->last_tx_time = GetRAK4630Time();  // now        
        // only update relayby_id if relayto_id was us.        
        if ( relayto_id == gLHLocalDID ){
            pnode->rev_relay_id = relayby_id;
            pnode->relay_hops = relay_hops;
        }
#endif
    }
    //DebugDumpNodes();
}

#ifdef LOWHOP_SENSOR
// @brief a system clock change occured, fixup datetimes
void UpdateLHDatetimes( const std::time_t time_offset ){
    gLHNodes.OnClockChange(time_offset);
}

// @brief maintain node & network status and state
// @returns false if we no longer have an uplink to Base
bool UpdateLHNetworkState(){
    TSLogs << "DBG UpdateLHNetworkState\n";
    // remove old nodes
    if ( gLHNodes.RemoveOldNodes() ){
        TSLogs << "DBG old nodes were removed\n";
        // nodes removed. is our uplink still valid?
        if ( gLHUplink.IsValid() && !FindLHNodeInfo(gLHUplink.GetUplink()) ){
            // uplink was removed. we have no uplink.
            TSLogs << "DBG Uplink was removed. No Uplink\n";
            gLHUplink.SetInvalid();
        }
    }

    // uplink still stable?
    if ( gLHUplink.IsValid() ){
        // find the nodeinfo, check if base_hops changed
        LHNodeInfo_t *pnode = FindLHNodeInfo(gLHUplink.GetUplink());
        if ( !pnode ){
            TSLogs << "ERR Uplink not found in NodeArray! " << gLHUplink.GetUplink() <<"\n";
            gLHUplink.SetInvalid();
        }
        else if (!gLHUplink.IsBase() && ((pnode->base_hops+1) != gLHUplink.GetBaseHopCount()) ){
            TSLogs << "DBG Uplink " << gLHUplink.GetUplink() << " base_hops was "
                << gLHUplink.GetBaseHopCount() << " now it is " << pnode->base_hops << "\n";
            gLHUplink.SetInvalid();
        }
    }

    DebugDumpNodes(); //debug
    TSLogs << "DBG Uplink " << gLHUplink.GetUplink() << " VALID flag: " << gLHUplink.IsValid() << "\n";
    return gLHUplink.IsValid();
}

bool IsLHUplink( const LHDeviceAddress_t &dev_id ){
    if ( !gLHUplink.IsValid() )
        return false;

    return gLHUplink.GetUplink() == dev_id;
}

bool IsLHUplinkValid(){
    return gLHUplink.IsValid();
}

// @brief Use node as our uplink
bool SetLHUplink(const LHDeviceAddress_t &dev_id){
    TSLogs << "DBG SetLHUplink " << dev_id << "\n";
/*
    // if preferred uplink is set and healthy/available use it
    if ( !gLHUplink.GetPreferred().IsNull() ){
        LHNodeInfo_t *pnode = FindLHNodeInfo(gLHUplink.GetPreferred());
        if ( pnode && pnode->base_hops != 0 ){
            gLHUplink.Set( pnode->dev_id, pnode->base_hops+1 );
            TSLogs << "DBG Choosing preferred uplink " << gLHUplink.GetUplink() << " with "
                << gLHUplink.GetBaseHopCount() << " hops.\n";
            return true;    // success
        }
    }
*/
    // see if passed in node is a valid choice for us to use as an uplink
    if ( dev_id.IsBase() ){
        gLHUplink.Set(dev_id, 1);
    }
    else{
        LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
        if ( !pnode )
            return false;

        if ( pnode->base_hops == 0 )
            return false;

        gLHUplink.Set(dev_id, pnode->base_hops+1);
    }

    TSLogs << "DBG using uplink " << gLHUplink.GetUplink() << " with " << gLHUplink.GetBaseHopCount()
        << " hops.\n";
    return true; // success
}

#endif
#ifdef LOWHOP_BASE
// @brief Manually add this node id to node array if not already -
void AddLHNode(const LHNodeInfo_t &node_info){
    // is node known to us? find it
    LHNodeInfo_t *pnode = FindLHNodeInfo(node_info.dev_id);
    if ( pnode ){
        TSLogs.Time() << fmt::format("DEBUG LH Add New Node {} Already Exists!!\n", node_info.dev_id);
        *pnode = node_info;
        return;
    }
    TSLogs.Time() << fmt::format("LH Add New Node {}\n", node_info.dev_id);
    gLHNodes.push_back(node_info); // add the node to array
    //UpdateLHNodeInfo(dev_id, dev_id, 0x00000000, 0, 0, 0, false);
    //pnode = FindLHNodeInfo(dev_id);
    //assert( pnode != nullptr );
}

// @brief add command to node's outgoing queue
// @pass bool frem_dupes true to remove duplicate already queued commands
void QueueLHNodeCmd(LHQueuedCommand_t &qcmd, bool frem_dupes){
    // is node known to us? find it
    LHNodeInfo_t *pnode = FindLHNodeInfo(qcmd.dest_id);
    if ( !pnode ){
        // a node we haven't heard from yet. new node.
        UpdateLHNodeInfo(qcmd.dest_id, qcmd.dest_id, 0x00000000, 0, 0, 0);
        pnode = FindLHNodeInfo(qcmd.dest_id);
        assert( pnode != nullptr );
    }

    // remove any existing duplicate commands
    if ( frem_dupes ){
        for ( auto icmd = pnode->cmd_queue.begin(); icmd != pnode->cmd_queue.end();){
            if ( icmd->pkt_type == qcmd.pkt_type ){
                icmd = pnode->cmd_queue.erase(icmd);
                TSLogs.Time() << fmt::format("$$ LH Removed duplicate qcmd node {} pkt_type ({}) {}\n",
                                             pnode->dev_id, (uint8_t)qcmd.pkt_type, GetLHPktTypeName(qcmd.pkt_type));
            }
            else{
                ++icmd;
            }
        }
    }

    // limit cmdq size by removing oldest cmd
    while ( pnode->cmd_queue.size() > LOWHOP_CMD_MAX_QCMDS ){
        TSLogs.Time() << fmt::format("LH MAX QCMDS reached. Remove Queued command for node {} name {}\n",
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

    TSLogs.Time() << fmt::format("LH Queued command for node {} name {} fsend_now={} fw_name {} last_txtime_t {} cmdq size {}\n",
        pnode->dev_id, pnode->name, qcmd.fsend_now, qcmd.fw_path.filename().string(), pnode->last_tx_time, pnode->cmd_queue.size());
}
#endif

// @brief lookup name of a node
LHStrClass GetLHNodeName(const LHDeviceAddress_t &dev_id) {
    LHStrClass name;
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if (pnode && pnode->name.length() > 0)
        name = pnode->name;
    else
        name = "{unknown}";
    return name;
}

// @brief get tlm_offset_ms of a node
uint32_t GetLHNodeTlmOffsetMs(const LHDeviceAddress_t &dev_id){
    uint32_t toff_ms = 0;
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        toff_ms = pnode->tlm_offset_ms;
    return toff_ms;
}

// @brief set node's protocol versions
void SetLHNodeVersion(const LHDeviceAddress_t &dev_id, uint16_t lh_ver, uint16_t fw_ver, uint32_t bl_ver) {
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode ){
        pnode->protocol_ver = lh_ver;
        pnode->firmware_ver = fw_ver;
        pnode->bootloader_ver = bl_ver;
    }
}

// @brief set name of a node
void SetLHNodeName(const LHDeviceAddress_t &dev_id, const LHStrClass &node_name) {
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if (pnode) {
        pnode->name = node_name;
    }
}

// @brief set node's latest reported base hop count
void SetLHNodeBaseHopCount(const LHDeviceAddress_t &dev_id, uint16_t base_hops) {
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if (pnode) {
        pnode->base_hops = base_hops;
    }
}

/*
// @brief set node's Last TX time as now
void SetLHNodeLastTlmTime(const LHDeviceAddress_t &dev_id ){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode ){
        pnode->last_tx_time = std::time( nullptr ); // now!
    }
}
*/
#ifdef LOWHOP_BASE
// @brief get preferred uplink of a node
LHDeviceAddress_t GetLHNodePreferredUplink( const LHDeviceAddress_t &dev_id ){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    LHDeviceAddress_t prf_uplink_id;
    if (pnode) {
        prf_uplink_id = pnode->preferred_uplink_id;
    }
    return prf_uplink_id;
}

// @brief set preferred uplink of a node
void SetLHNodePreferredUplink(const LHDeviceAddress_t &dev_id, const LHDeviceAddress_t &prf_uplink_id) {
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if (pnode) {
        pnode->preferred_uplink_id = prf_uplink_id;
    }
}

// @brief get wake_count of a node
uint32_t GetLHNodeWakeCount(const LHDeviceAddress_t &dev_id){
    uint32_t wakec = 0;
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        wakec = pnode->wake_count;
    return wakec;
}

// @brief get fw_offset of a node
uint32_t GetLHNodeFWOffset(const LHDeviceAddress_t &dev_id){
    uint32_t fw_offset = 0;
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        fw_offset = pnode->fw_offset;
    return fw_offset;
}

std::filesystem::path GetLHNodeFWPath(const LHDeviceAddress_t &dev_id){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        return pnode->fw_path;
    return {};
}

void SetLHNodeFWOffset(const LHDeviceAddress_t &dev_id, uint32_t fw_offset){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        pnode->fw_offset = fw_offset;
}

void ClearLHNodeFWPath(const LHDeviceAddress_t &dev_id ){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode )
        pnode->fw_path.clear();
}

// @brief store node's measuremnt interval
void SetLHNodeInterval(const LHDeviceAddress_t &dev_id, int32_t measurement_ms ){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode ){
        pnode->measurement_ms = measurement_ms;
    }
}

// @brief store node's last replyradarconfig payload
void SetLHNodeLastRadarCfg(const LHDeviceAddress_t &dev_id,
                           const PayloadReplyRadarConfig_t &radar_cfg ){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode ){
        pnode->radar_cfg = radar_cfg;
    }
}

// @brief store node's last reported wake_count
void SetLHNodeWakeCount(const LHDeviceAddress_t &dev_id, uint32_t wake_count){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( pnode ){
        pnode->wake_count = wake_count;
    }
}

bool IsRXWindowOpen(){
    // The sensor node RX window starts then the Wake Timer triggers and lasts for the wake interval.
    //
    // Compensate for LOWHOP_TX_EXTRA_WAIT_SEC..
    // When nodes wake up they send telemetry, scheduled one after another.
    // Give that a chance to finish before we start sending commands etc..
    int32_t total_tlm_wait_time = LOWHOP_TX_EXTRA_WAIT_SEC; // todo: calc this total from gLHNodes array
    int32_t wake_time_elapsed = gLHWakeTimer.GetSeconds();
    if ( wake_time_elapsed > total_tlm_wait_time && wake_time_elapsed <= LOWHOP_RX_WINDOW_SEC ){
        //TSLogs.Time() << "**** Sensor RXWindow is open! **** TotalTlmWait was " << total_tlm_wait_time << "s\n";
        return true;
    }

    return false;
}

// limit historyq size
void LimitLHHistQSize( LHNodeInfo_t &node ){
    while ( node.cmd_hist.size() > LOWHOP_CMD_MAX_QCMDS ){
        TSLogs.Time() << fmt::format("LH MAX History QCMDS reached. Remove history command for node {}\n",
            node.dev_id );
        node.cmd_hist.pop_front();
    }
}

// @brief Look at all nodes and all queued commands. Which one needs to send soonest (if any)?
//        This command will be sent next.
bool NextLHQCmd(LHQueuedCommand_t &qnowcmd){
    //std::time_t nowt = std::time(nullptr);
    // we are searching. this variable tracks what we find to be the best node.
    LHNodeInfo_t *pqnow_node = nullptr;

    // check all nodes
    for ( auto &node : gLHNodes ){
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

        int max_retry_count = LOWHOP_CMD_MAX_RETRY;
        // special case for CMD_RESET, we just wait for 1 try.
        // because if the sensor reset right away we wont get any reply and we don't want to repeat a reboot command.
        if ( qnowcmd.pkt_type == PacketType_t::CMD_RESET ){
            max_retry_count = 1;
        }

        // too many retries? move it to hist
        if ( pqnow_node->reply_wait_count >= max_retry_count ){
            TSLogs.Time() << fmt::format("LH Command MAX_RETRY {} qcmd pkt_type ({}) {}\n",
                pqnow_node->dev_id, (uint8_t)qnowcmd.pkt_type, GetLHPktTypeName(qnowcmd.pkt_type));
            // move qcmd to history
            pqnow_node->cmd_queue.pop_front();
            pqnow_node->cmd_hist.push_back(qnowcmd);
            LimitLHHistQSize( *pqnow_node );
            // reset node wait count
            pqnow_node->reply_wait_count = 0;

            if ( gLHCallback.OnReplyTimeout )
                gLHCallback.OnReplyTimeout(pqnow_node->dev_id);

            return false; // command timed out, removed it
        }
        else if (pqnow_node->reply_wait_count > 0) {
            TSLogs.Time() << fmt::format("LH Command RETRY {} #{} qcmd pkt_type ({}) {}\n",
                pqnow_node->dev_id, pqnow_node->reply_wait_count,
                (uint8_t)qnowcmd.pkt_type, GetLHPktTypeName(qnowcmd.pkt_type));
        }
        ++pqnow_node->reply_wait_count;
        pqnow_node->reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
        return true;
    }
    // no queued commands ready
    return false;
}

// @brief remove the node's 'active'/current command even if we havent waited for a reply
void IncrementLHCmdQ(const LHDeviceAddress_t &dev_id){
    LHNodeInfo_t *pnode = FindLHNodeInfo(dev_id);
    if ( !pnode )
        return; // unknown node, do nothing

    // move qcmd to history
    auto qcmd = pnode->cmd_queue.front();
    pnode->cmd_queue.pop_front();
    pnode->cmd_hist.push_back(qcmd);
    LimitLHHistQSize( *pnode );

    // reset reply/retry status
    pnode->reply_wait_count = 0;
    pnode->reply_status = PacketReplyStatus_t::RSTATUS_UNSET;
}

// @brief check node's cmd_queue for expired commands
void RemoveLHQOldCmds(){
    std::time_t nowt = std::time(nullptr);
    // check all nodes
    for ( auto &node : gLHNodes ){
        // any commands queued?
        if ( node.cmd_queue.size() > 0 ){
            int32_t since_last_txtime = difftime(nowt,node.last_tx_time);
            if ( since_last_txtime > LOWHOP_CMD_EXPIRE_SEC ){
                // remove one command from queue
                LHQueuedCommand_t qcmd = node.cmd_queue.front();
                node.cmd_queue.pop_front();
                TSLogs.Time() << fmt::format("LH Removed expired qcmd node {} pkt_type {}\n", node.dev_id, (uint8_t)qcmd.pkt_type);
            }
        }
        // limit histq size also
        LimitLHHistQSize(node);
    }
}

// @brief See if we know how to relay a packet to this destination.
// @returns true if we can and sets relay did in relay_to
bool GetLHNextHopFor(const PacketDeviceID_t &dest_id, LHDeviceAddress_t &relay_to) {
    LHDeviceAddress_t dest_addr( dest_id );
    // is destination a base!?!
    if (dest_id.IsBase()) {
        TSLogs << "No relay found to base " << dest_addr << ", we are BASE!\n";
        return false;
    }
    // else see if we have info on this destination node.
    else {
        LHNodeInfo_t *pnode = FindLHNodeInfo(dest_addr);  // gLHNodes.Find(dest_addr);
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

#ifdef LOWHOP_SENSOR
// @brief See if we know how to relay a packet to this destination.
// @returns true if we can and sets relay did in relay_to
bool GetLHNextHopFor(const PacketDeviceID_t &dest_id, LHDeviceAddress_t &relay_to) {
    LHDeviceAddress_t dest_addr( dest_id );
    // is destination a base?
    if (dest_id.IsBase()) {
        // relay via uplink if set
        if ( gLHUplink.IsValid() ){
            relay_to = gLHUplink.GetUplink();
            TSLogs << "Have route to Base " << dest_addr << " via uplink " << relay_to << " hop count " 
                << gLHUplink.GetBaseHopCount() << "\n";
            return true; // success
        }
        // no uplink to base
        else{
            TSLogs << "No uplink to base " << dest_addr << ".\n";
            // is there a preferred uplink? and is it usable?
            if ( gLHNodes.IsGoodHopToBase( gLHUplink.GetPreferred() ) ){
                relay_to = gLHUplink.GetPreferred();
                TSLogs << "DBG Trying preferred uplink " << relay_to << " to base\n";
                return true; // success
            }
            // no, find best hop if any
            LHNodeInfo_t best_hop;
            auto bhop_count = gLHNodes.FindBestHopToBase(best_hop);
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
        LHNodeInfo_t *pnode = gLHNodes.Find(dest_addr);
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
uint16_t GetLHBaseHops() {
    return gLHUplink.IsValid() ? gLHUplink.GetBaseHopCount() : 0;
}

// our uplink if any
LHDeviceAddress_t GetLHUplinkAddr(){
    return gLHUplink.GetUplink();
}

// preferred uplink if any
LHDeviceAddress_t GetLHPreferredUplinkAddr(){
    return gLHUplink.GetPreferred();
}

#endif

/*
// @brief Determine if this is a command Reply packet
bool IsLHReplyPkt(PacketType_t pkt_type) {
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
bool IsLHPktToUs( const PacketHeader_t &pkt_hdr ){
    bool fthis_base = gfLHIsBase && gLHLocalDID.IsBase() && pkt_hdr.RelayTo.IsBase() && pkt_hdr.Dest.IsBase();
    bool fthis_dest = gLHLocalDID.IsMatch(pkt_hdr.RelayTo) && gLHLocalDID.IsMatch(pkt_hdr.Dest);
    return fthis_base || fthis_dest || pkt_hdr.Dest.IsBroadcast();
}

#ifdef LOWHOP_BASE
// @brief special processing when we RX a cmd reply packet
template<class TPacket>
bool ProcessLHReplyPkt(TPacket *ppkt, uint16_t size){
    // is the node known?
    LHNodeInfo_t *pnode = FindLHNodeInfo(ppkt->Header.Src);
    if ( !pnode ){ // not known? do nothing
        TSLogs.Time() << fmt::format("LH Command Reply from unknown node {} ignored.\n",
                                     pnode->dev_id);
        return false;
    }
    // does the reply match our 'active' command? The 'active' command is the front() of the cmd_queue.
    if ( pnode->cmd_queue.empty() ){
        TSLogs.Time() << fmt::format("LH Command Reply but cmd_queue is empty for node {}.\n",
                                     pnode->dev_id);
        return false;
    }
    LHQueuedCommand_t qcmd = pnode->cmd_queue.front();
    if ( ppkt->RefID != qcmd.ref_id ){
        TSLogs.Time() << fmt::format("LH Command Reply ref_id didn't match for node {}. pkt {} qcmd {}\n",
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

    std::string rstatus_str = GetLHPktReplyStatusName(reply_status);
    TSLogs.Time() << fmt::format("LH Command Reply {} {} err {}\n",
        pnode->dev_id, rstatus_str, error_code);

    // call user callback
    if ( gLHCallback.OnReplyCmd )
        gLHCallback.OnReplyCmd(pnode->dev_id, reply_type, reply_status);

    return true;
}
#endif

// @brief validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckLHPkt(PacketHeader_t *ppkt_hdr, uint16_t size, void (*OnPacket)(TPacket *)) {
    TPacket *ppkt = reinterpret_cast<TPacket *>(ppkt_hdr);
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "LH invalid packet size\n";
        return false;
    }

    if ( !IsLHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "LH Packet not for us. ignored.\n";
        return false;
    }

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}

#ifdef LOWHOP_SENSOR
// @brief For Telemetry packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckLHTelemetryPkt(PacketHeader_t *ppkt_hdr, uint16_t size, void (*OnPacket)(TPacket *)) {
    TPacket *ppkt = reinterpret_cast<TPacket *>(ppkt_hdr);
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "LH invalid telemetry packet size\n";
        return false;
    }
    // we accept all Telemetry packets even if not for us.
    // we use our neighbors wake/telemetry packet to learn base hops and interval.
    //
    // special handling for Telemetry BEFORE OnPacket
    // update base hop then call generic Telemetry user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeBaseHopCount(dev_id, ppkt->BaseHops);
    TSLogs << "Sniffed base_hops for " << dev_id << " hop count " << ppkt->BaseHops << "\n";
    
    if ( gLHCallback.OnTelemetry )
        gLHCallback.OnTelemetry( dev_id, ppkt->BaseHops );

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}
#endif

#ifdef LOWHOP_BASE
// @brief For Telemetry packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckLHTelemetryPkt(PacketHeader_t *ppkt_hdr, uint16_t size, void (*OnPacket)(TPacket *)) {
    TPacket *ppkt = reinterpret_cast<TPacket *>(ppkt_hdr);
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "LH invalid telemetry packet size\n";
        return false;
    }

    if ( !IsLHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "LH Telemetry Packet not for us. ignored.\n";
        return false;
    }

    // special handling for Telemetry BEFORE OnPacket
    // update wake count and base hop then call generic Telemetry user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeBaseHopCount(dev_id, ppkt->BaseHops);
    SetLHNodeWakeCount(dev_id, ppkt->Payload.wake_count);
    
    if ( gLHCallback.OnTelemetry )
        gLHCallback.OnTelemetry( dev_id, ppkt->BaseHops );

    // call the callback for pkt_type
    if (OnPacket == NULL)  // no callback then do nothing
        return true;

    OnPacket(ppkt);
    return true;
}

// @brief For Reply packets, validate RX packet size and do callbacks
// @return true on success
template<class TPacket>
bool CheckLHReplyPkt(PacketHeader_t *ppkt_hdr, uint16_t size, void (*OnPacket)(TPacket *)) {
    TPacket *ppkt = reinterpret_cast<TPacket *>(ppkt_hdr);
    if (size != sizeof(TPacket)) {  // invalid pkt size? fail
        TSLogs << "LH invalid reply packet size\n";
        return false;
    }

    if ( !IsLHPktToUs( ppkt->Header ) ){
        TSLogs.Time() << "LH Reply Packet not for us. ignored.\n";
        return false;
    }

    // special handling for replies BEFORE OnPacket
    if ( !ProcessLHReplyPkt( ppkt, size ) ){
        TSLogs << "ProcessLHReplyPkt failed!\n";
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
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeVersion(dev_id, ppkt->Payload.lh_version, ppkt->Payload.fw_version, ppkt->Payload.bl_version );

    if ( gLHCallback.OnReplyVersion )
        gLHCallback.OnReplyVersion(ppkt);
}

void InterceptOnReplyName(PacketReplyName_t *ppkt){
    // save name then do user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeName(dev_id, ppkt->Payload.name.Get());

    if ( gLHCallback.OnReplyName )
        gLHCallback.OnReplyName(ppkt);
}

void InterceptOnReplyPreferredUplink(PacketReplyPreferredUplink_t *ppkt){
    // save uplink then do user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    LHDeviceAddress_t prf_uplink_id(ppkt->Payload.uplink);
    SetLHNodePreferredUplink(dev_id, prf_uplink_id);

    if ( gLHCallback.OnReplyPreferredUplink )
        gLHCallback.OnReplyPreferredUplink(ppkt);
}

void InterceptOnReplyInterval(PacketReplyInterval_t *ppkt){
    // save interval then do user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeInterval(dev_id, ppkt->Payload.measurement_ms);

    if ( gLHCallback.OnReplyInterval )
        gLHCallback.OnReplyInterval(ppkt);
}

void InterceptOnReplyRadarConfig(PacketReplyRadarConfig_t *ppkt){
    // save rcfg then do user callback if any
    LHDeviceAddress_t dev_id(ppkt->Header.Src);
    SetLHNodeLastRadarCfg(dev_id, ppkt->Payload);

    if ( gLHCallback.OnReplyRadarConfig )
        gLHCallback.OnReplyRadarConfig(ppkt);
}
#endif
#ifdef LOWHOP_SENSOR
void InterceptOnCommandGetPreferredUplink(PacketCommandGetPreferredUplink_t *ppkt){
    //LHDeviceAddress_t dev_id(ppkt->Header.Src);
    TSLogs << "LH Get preferred uplink: " << gLHUplink.GetPreferred() << "\n";

    if ( gLHCallback.OnCommandGetPreferredUplink )
        gLHCallback.OnCommandGetPreferredUplink(ppkt);
}

void InterceptOnCommandSetPreferredUplink(PacketCommandSetPreferredUplink_t *ppkt){
    //LHDeviceAddress_t dev_id(ppkt->Header.Src);
    if ( !gLHLocalDID.IsMatch( ppkt->Payload.uplink ) ) // can't be self
        gLHUplink.SetPreferred( ppkt->Payload.uplink );
        
    TSLogs << "LH Set preferred uplink: " << gLHUplink.GetPreferred() << "\n";
    // Do the callback BEFORE changing current uplink.
    // So the reply to this command uses existing uplink.
    if ( gLHCallback.OnCommandSetPreferredUplink )
        gLHCallback.OnCommandSetPreferredUplink(ppkt);

    // NOW, if we have an uplink set that isn't the preferred uplink
    if ( !gLHUplink.IsPreferred() ){
        // unset the uplink to give it a chance to be chosen.
        gLHUplink.SetInvalid();
    }
}
#endif

// check some things and see if this packet can be rejected
bool IsLHPktValid(PacketHeader_t *ppkt_hdr, uint16_t size){
    // check for garbage addresses
    if ( !ppkt_hdr->Src.IsKnownOUI() ||
         !ppkt_hdr->Dest.IsKnownOUI() ||
         !ppkt_hdr->RelayBy.IsKnownOUI() ||
         !ppkt_hdr->RelayTo.IsKnownOUI() ){
        return false;
    }

    if ( ppkt_hdr->HopCount > LOWHOP_MAX_HOP_COUNT ){
        TSLogs << "LH Invalid Pkt: HopCount maxed out.\n";
        return false;   // this packet is going in circles or had gone too far!
    }

    if ( ppkt_hdr->Dest.IsMatch( ppkt_hdr->Src ) ){
        TSLogs << "LH Invalid Pkt: from self to self.\n";
        return false;   // from/to self
    }

    if ( ppkt_hdr->RelayTo.IsMatch(ppkt_hdr->Src) ){
        TSLogs << "LH Invalid Pkt: relaying to self.\n";
        return false;   // from self back to self
    }

    if ( ppkt_hdr->RelayBy.IsMatch(ppkt_hdr->Dest) ){
        TSLogs << "LH Invalid Pkt: relaying away from self.\n";
        return false;   // relay away from dest
    }

    if ( ppkt_hdr->Src.IsMatch(gLHLocalDID.ToDID()) ){
        TSLogs << "LH Invalid Pkt: we RXed our own packet.\n";
        return false;   // RX our own packet!
    }

    return true;
}

// @brief RX new packet, determine packet type and call processing function
// @return true on succeess
bool ProcessLHPkt(PacketHeader_t *ppkt_hdr, uint16_t size, int16_t this_rssi, int16_t this_snr) {
    // we have a chance to reject some garbage packets here..
    if ( !IsLHPktValid(ppkt_hdr,size) ){
        TSLogs << "RX Pkt failed validity check. Ignoring.\n";
        return false;
    }

    // is packet for us? are we a base..
    if (gfLHIsBase) {
        // Is packet for us or is it a broadcast?
        // We may hear distant relayed packets that are dest us, but relayed to another node.
        // This check blocks those.
        bool fbase_dest = ppkt_hdr->RelayTo.IsBase() && ppkt_hdr->Dest.IsBase();
        if (!fbase_dest && !ppkt_hdr->Dest.IsBroadcast()) {
            TSLogs.Time() << "LH RX LHPacket not for us. ignored.\n";
            return false;
        }

        // save the talking node's info. This is how we discover nodes trying to reach us directly.
        LHDeviceAddress_t src_id(ppkt_hdr->Src);
        LHDeviceAddress_t relayby_id(ppkt_hdr->RelayBy);
        LHDeviceAddress_t relayto_id(ppkt_hdr->RelayTo);
        UpdateLHNodeInfo(src_id, relayby_id, relayto_id, ppkt_hdr->HopCount, this_rssi, this_snr);
    }
    // else we are a sensor node
    else {
        // save the talking node's info. This is how we discover all neighboring nodes.
        LHDeviceAddress_t src_id(ppkt_hdr->Src);
        LHDeviceAddress_t relayby_id(ppkt_hdr->RelayBy);
        LHDeviceAddress_t relayto_id(ppkt_hdr->RelayTo);
        UpdateLHNodeInfo(src_id, relayby_id, relayto_id, ppkt_hdr->HopCount, this_rssi, this_snr);

#ifdef LOWHOP_SENSOR
        // are we being asked to relay this packet? do it
        if ( !gLHLocalDID.IsMatch(ppkt_hdr->Dest) && gLHLocalDID.IsMatch(ppkt_hdr->RelayTo) ){
            // ..well not if we don't have a synchronized uplink yet. we dont want to contribute
            // to wrong packet relaying. Makes noise and drains battery.
            if ( !gLHUplink.IsValid() ){
                TSLogs.Time() << "DBG LH RX we can't relay yet.\n";
                return false;
            }            

            if ( gLHCallback.OnRelayPkt ){
                gLHCallback.OnRelayPkt( ppkt_hdr, size );
                return true; // done with this pkt
            }
            TSLogs << "ERR No user callback for relaying!\n";
            return false; // no user callback for relay is strange! fail.
        }
#endif
    }

    // check size & do callbacks - basically a manual dispatcher
    uint8_t *payload = NULL;
    switch (ppkt_hdr->PktType) {
#ifdef LOWHOP_SENSOR
        // Any relaying happened above. This packet was directly heard/RX.
        //
        // Command packets
        case PacketType_t::CMD_RESET:
            return CheckLHPkt<PacketCommandReset_t>(ppkt_hdr, size, gLHCallback.OnCommandReset);
            break;

        case PacketType_t::CMD_GET_VERSION:
            return CheckLHPkt<PacketCommandGetVersion_t>(ppkt_hdr, size, gLHCallback.OnCommandGetVersion);
            break;

        case PacketType_t::CMD_GET_NAME:
            return CheckLHPkt<PacketCommandGetName_t>(ppkt_hdr, size, gLHCallback.OnCommandGetName);
            break;

        case PacketType_t::CMD_SET_NAME:
            return CheckLHPkt<PacketCommandSetName_t>(ppkt_hdr, size, gLHCallback.OnCommandSetName);
            break;

        case PacketType_t::CMD_GET_INTERVAL:
            return CheckLHPkt<PacketCommandGetInterval_t>(ppkt_hdr, size, gLHCallback.OnCommandGetInterval);
            break;

        case PacketType_t::CMD_SET_INTERVAL:
            return CheckLHPkt<PacketCommandSetInterval_t>(ppkt_hdr, size, gLHCallback.OnCommandSetInterval);
            break;

        case PacketType_t::CMD_CONFIG_LORA:
            return CheckLHPkt<PacketCommandConfigLoRa_t>(ppkt_hdr, size, gLHCallback.OnCommandConfigLoRa);
            break;

        case PacketType_t::CMD_CONFIG_RADAR:
            return CheckLHPkt<PacketCommandConfigRadar_t>(ppkt_hdr, size, gLHCallback.OnCommandConfigRadar);
            break;

        case PacketType_t::CMD_GET_RADAR_CONFIG:
            return CheckLHPkt<PacketCommandGetRadarConfig_t>(ppkt_hdr, size, gLHCallback.OnCommandGetRadarConfig);
            break;

        case PacketType_t::CMD_BEGIN_DFU_UPLOAD:
            return CheckLHPkt<PacketCommandBeginDFUUpload_t>(ppkt_hdr, size, gLHCallback.OnCommandBeginDFUUpload);
            break;

        case PacketType_t::CMD_DFU_UPLOAD:
            return CheckLHPkt<PacketCommandDFUUpload_t>(ppkt_hdr, size, gLHCallback.OnCommandDFUUpload);
            break;

        case PacketType_t::CMD_VERIFY_DFU_UPLOAD:
            return CheckLHPkt<PacketCommandVerifyDFUUpload_t>(ppkt_hdr, size, gLHCallback.OnCommandVerifyDFUUpload);
            break;

        case PacketType_t::CMD_BLINK_LED:
            return CheckLHPkt<PacketCommandBlinkLED_t>(ppkt_hdr, size, gLHCallback.OnCommandBlinkLED);
            break;

        case PacketType_t::CMD_GET_PREFERRED_UPLINK:
            return CheckLHPkt<PacketCommandGetPreferredUplink_t>(ppkt_hdr, size, InterceptOnCommandGetPreferredUplink);
            break;

        case PacketType_t::CMD_SET_PREFERRED_UPLINK:
            return CheckLHPkt<PacketCommandSetPreferredUplink_t>(ppkt_hdr, size, InterceptOnCommandSetPreferredUplink);
            break;
#endif
#ifdef LOWHOP_BASE
        // reply packets
        case PacketType_t::REPLY_VERSION:
            return CheckLHReplyPkt<PacketReplyVersion_t>(ppkt_hdr, size, InterceptOnReplyVersion);
            break;

        case PacketType_t::REPLY_SUCCESS:
            return CheckLHReplyPkt<PacketReplySuccess_t>(ppkt_hdr, size, gLHCallback.OnReplySuccess);
            break;

        case PacketType_t::REPLY_FAILURE:
            return CheckLHReplyPkt<PacketReplyFailure_t>(ppkt_hdr, size, gLHCallback.OnReplyFailure);
            break;

        case PacketType_t::REPLY_NAME:
            return CheckLHReplyPkt<PacketReplyName_t>(ppkt_hdr, size, InterceptOnReplyName);
            break;

        case PacketType_t::REPLY_PREFERRED_UPLINK:
            return CheckLHReplyPkt<PacketReplyPreferredUplink_t>(ppkt_hdr, size, InterceptOnReplyPreferredUplink);
            break;

        case PacketType_t::REPLY_INTERVAL:
            return CheckLHReplyPkt<PacketReplyInterval_t>(ppkt_hdr, size, InterceptOnReplyInterval);
            break;

        case PacketType_t::REPLY_RADAR_CONFIG:
            return CheckLHReplyPkt<PacketReplyRadarConfig_t>(ppkt_hdr, size, InterceptOnReplyRadarConfig);
            break;

        case PacketType_t::REPLY_DFU_UPLOAD:
            return CheckLHReplyPkt<PacketReplyDFUUpload_t>(ppkt_hdr, size, gLHCallback.OnReplyDFUUpload);
            break;

        case PacketType_t::REPLY_VERIFY_DFU_UPLOAD:
            return CheckLHReplyPkt<PacketReplyVerifyDFUUpload_t>(ppkt_hdr, size, gLHCallback.OnReplyVerifyDFUUpload);
#endif

        // telemetry packets
        case PacketType_t::RADAR_TELEMETRY:
            return CheckLHTelemetryPkt<PacketRadarTelemetry_t>(ppkt_hdr, size, gLHCallback.OnRadarTelemetry);
            break;

        case PacketType_t::RADAR_GNSS_TELEMETRY:
            return CheckLHTelemetryPkt<PacketRadarGNSSTelemetry_t>(ppkt_hdr, size, gLHCallback.OnRadarGNSSTelemetry);
            break;

        default:
            // unknown packet type
            return false;
    }

    return true;  // success
}
