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

/// radar sensor network webservice
///
/// todo: if we ever allow loraconfig changes we have to recalc telemetry offsets,
/// because that changes packet times.

#define CROW_STATIC_DIRECTORY "./sensor-data/"
#define CROW_STATIC_ENDPOINT "/sensor-data/<path>"
#include <crow.h>
#include <filesystem>
#include <fmt/format.h>

#include "loracomms.h"

std::future<void> gWebSvcFuture;
crow::SimpleApp gCrowApp;
std::filesystem::path FIRMWARE_PATH("./sensor-fw/");

bool IsSubPath(const std::filesystem::path &base, const std::filesystem::path &target){
    auto canonical_base = std::filesystem::weakly_canonical(base);
    auto canonical_target = std::filesystem::weakly_canonical(target);

    auto mismatch = std::mismatch(canonical_base.begin(), canonical_base.end(), canonical_target.begin());
    return mismatch.first == canonical_base.end();
}

std::vector<std::filesystem::path> GetFWList(){
    std::vector<std::filesystem::path> fw_list;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(FIRMWARE_PATH, ec)){
        if (!entry.is_directory() && entry.path().extension() == ".bin"){
            fw_list.push_back(entry.path());
            //debug std::cout << "Found FW File: " << entry.path().filename() << "\n";
        }
    }
    if ( ec ){
        std::cout << "Error in GetFWList: " << ec.message() << "\n";
    }
    return fw_list;
}

#include "webservice_templates.h"

std::string RenderStatusPage(){
    std::stringstream page_text;
//            <meta http-equiv="refresh" content="30">
    page_text << R"(
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <title>sensor network status</title>
            <meta charset="UTF-8">
            <style>
                div p {
                    font-family: 'Monaco', 'Menlo', 'Consolas', 'Courier New';
                }
                table tr th, table tr td {
                    text-align: left;
                    padding-right: 24px;

                }
                input:not([type="checkbox"]) {
                    min-width: 100px;
                }
                #tlm-column-names {
                    font-size: 0.75em; /* or any other value */
                }
                textarea {
                    width: 90%;
                    height: 275px;
                    white-space: pre-wrap; /* Ensures line breaks are respected */
                    overflow-y: scroll;
                    resize: none;
                    font-size: 1em;
                    padding: 10px;
                    background-color: rgb(220,220,220);
                }
                .container {
                    display: grid;
                    grid-template-columns: repeat(2, 1fr); /* Create two equal columns */
                }
                .left-div, .right-div {
                    padding: 5px; /* Add some space between the divs and their borders */
                }
            </style>
        </head>
        <body>
    )";
    char host_name[256];
    gethostname(host_name, 255);
    page_text << "<h3>SENSOR NETWORK STATUS v" << BASE_FIRMWARE_VERSION_STR; // << "</h3>";
    page_text <<  " -- protocol v" << fmt::format("{:02X}", GetLHVersion());
    page_text <<  " -- hostname " << host_name << "</h3>";
    WSTemplates::GetHTML_NodeList(page_text);
    page_text << R"(
            <div id=tlm-column-names>
            <strong>CSV column names: (timestamp,sentence type,..tlm follows)</strong>
            <br>
    )";
    std::string tlm_column_names = GetTelemetryColumns();
    page_text << tlm_column_names;
    page_text << R"(
        </div>
    )";
    WSTemplates::GetHTML_RadarConfig(page_text);
    WSTemplates::GetHTML_GetSensorInfo(page_text);
    WSTemplates::GetHTML_SetInterval(page_text);
    WSTemplates::GetHTML_SetSensorName(page_text);
    WSTemplates::GetHTML_SetPreferredUplink(page_text);
    WSTemplates::GetHTML_BlinkLED(page_text);
    WSTemplates::GetHTML_ResetSensor(page_text);
    WSTemplates::GetHTML_UpdateSensorFW(page_text);
    page_text << R"(
        </div>
      </div>
      <br>
      <table>
        <tr><td><strong>Command Result:</td><td></strong><div id=result></div></td></tr>
      </table>
      <br>
    )";
    WSTemplates::GetHTML_BaseServerLog(page_text);
    page_text << R"(
      <script>
        function do_action_url(action_url){
            // Create a new XMLHttpRequest object.
            var xhr = new XMLHttpRequest();

            // Open the URL with values
            console.log(`action_url ${action_url}`);
            xhr.open('GET', action_url, true);
            xhr.responseType = "text"; // Explicitly expect plain text

            // Set up an event handler for when the request is complete.
            xhr.onload = function() {
                if (xhr.status === 200) {
                    console.log("Request successful");
                    document.getElementById("result").innerHTML = "Request successful - " + xhr.responseText;
                } else {
                    console.error('Failed to load page');
                    document.getElementById("result").innerHTML = "Failed - " + xhr.responseText;
                }
            };

            // Set up an event handler for when the request fails.
            xhr.onerror = function() {
                console.error('Error loading page');
                document.getElementById("result").innerHTML = "Error loading page";
            };

            // Send the request
            xhr.send();
        }

        // async load url and put resonse text in passed div id
        function update_log_ctrl(the_url){
            // Create a new XMLHttpRequest object.
            var xhr = new XMLHttpRequest();

            // Open the URL with values
            console.log(`the_url ${the_url}`);
            xhr.open('GET', the_url, true);
            xhr.responseType = "text"; // Explicitly expect plain text

            // Set up an event handler for when the request is complete.
            xhr.onload = function() {
                var log_ctrl = document.getElementById('server-log');
                if (xhr.status === 200) {
                    console.log("Request successful");
                    log_ctrl.value = xhr.responseText;
                } else {
                    console.error('Failed to load page');
                    log_ctrl.value =  "Failed - " + xhr.responseText;
                }
                log_ctrl.scrollTop = log_ctrl.scrollHeight;
            };

            // Set up an event handler for when the request fails.
            xhr.onerror = function() {
                console.error('Error loading page');
                var log_ctrl = document.getElementById('server-log');
                log_ctrl.value = "Error loading page";
            };

            // Send the request
            xhr.send();
        }
    )";
    WSTemplates::GetJS_GetSensorInfo(page_text);
    WSTemplates::GetJS_RadarConfig(page_text);
    WSTemplates::GetJS_NodeList(page_text);
    WSTemplates::GetJS_SetInterval(page_text);
    WSTemplates::GetJS_SetSensorName(page_text);
    WSTemplates::GetJS_SetPreferredUplink(page_text);
    WSTemplates::GetJS_BlinkLED(page_text);
    WSTemplates::GetJS_ResetSensor(page_text);
    WSTemplates::GetJS_UpdateSensorFW(page_text);
    WSTemplates::GetJS_BaseServerLog(page_text);
    page_text << R"(
        </script>
        </body>
        </html>
    )";

    return page_text.str();
}

// setup and start web service
bool StartWebService(){
    gCrowApp.loglevel(crow::LogLevel::Error);
    //// serve static file dir (see macro at beg. of file)
    //gCrowApp.add_static_dir();

    // setup routes
    //
    // main route
    CROW_ROUTE(gCrowApp, "/")([](){
        std::string status_page = RenderStatusPage();
        return status_page;
    });

    // get server log
    CROW_ROUTE(gCrowApp,"/server-log")
    ([](){
        return GetServerLog(50); // get the last X lines of server log
    });

    // recalculate all sensor node telemetry offset times
    CROW_ROUTE(gCrowApp,"/recalc-toffsets")
    ([](){
        RecalcTelemetryOffsets();
        return "Telemetry offsets recalculated. All nodes flagged for resync.";
    });

    // for debugging purposes, send a packet immediately to node
    CROW_ROUTE(gCrowApp,"/send-pktnow/<string>")
    ([](std::string dest_id_str){
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdSendPktNow(dest_id_dword);
        return "Send Pkt Now command queued for immediate send.";
    });

    // get node's last received radar configuration
    CROW_ROUTE(gCrowApp,"/get-last-radar-config/<string>")
    ([](std::string dest_id_str){
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        RadarConfiguration_t rcfg;
        if ( !GetLastRadarConfig(dest_id_dword, rcfg) ){
            memset(&rcfg,sizeof(rcfg), 0);
            rcfg.start_dist_mm = -1;
        }

        crow::json::wvalue rcfg_json({
            {"start_dist_mm",       rcfg.start_dist_mm},
            {"end_dist_mm",         rcfg.end_dist_mm},
            {"max_profile",         rcfg.max_profile},
            {"peak_sorting",        rcfg.peak_sorting},
            {"threshold_method",    rcfg.threshold_method},
            {"threshold_sensitivity", rcfg.threshold_sensitivity},
            {"fixed_amp_threshold", rcfg.fixed_amp_threshold},
            {"fixed_str_threshold", rcfg.fixed_str_threshold},
            {"signal_quality",      rcfg.signal_quality},
            {"max_step_count",      rcfg.max_step_count},
            {"reflector_shape",     rcfg.reflector_shape},
            {"close_range_leakage", rcfg.close_range_leakage}
        });

        return rcfg_json;
    });

    // request radar configuration
    CROW_ROUTE(gCrowApp,"/get-radar-config/<string>")
    ([](std::string dest_id_str){
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdGetRadarConfig(dest_id_dword);
        return "Get Radar Configuration command queued.";
    });

    // sensor log data
    CROW_ROUTE(gCrowApp,"/get-sensor-log/<string>")
    ([](const crow::request& req, crow::response& res, std::string dest_id_str){
        // compile ALL of this sensors logged data to a temporary file
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        std::filesystem::path slog_filepath;
        if ( !GetSensorLogFile( dest_id_dword, slog_filepath ) ){
            res.code = 500;
            res.write("Error getting sensor log file.");
            res.end();
        }

        // serve the file as download
        res.set_static_file_info_unsafe( slog_filepath.string() );
        res.set_header("Content-Disposition", "attachment; filename=\"" + slog_filepath.filename().string() + "\"");
        res.end();

        // can delete the file now
    });

    // configure radar
    CROW_ROUTE(gCrowApp,"/config-radar/<string>/<int>/<int>/<int>/<int>/<int>/<int>/<int>/<int>/<int>/<int>/<int>/<uint>")
    ([](std::string dest_id_str, int start_dist_mm, int end_dist_mm, int max_profile, int peak_sorting,
        int threshold_method, int threshold_sensitivity, int fixed_amp_threshold,
        int fixed_str_threshold, int signal_quality, int max_step_count, int reflector_shape,
        uint close_range_leakage ){

        RadarConfiguration_t rcfg;
        rcfg.start_dist_mm = start_dist_mm;
        rcfg.end_dist_mm = end_dist_mm;
        rcfg.max_profile = max_profile;
        rcfg.peak_sorting = peak_sorting;
        rcfg.threshold_method = threshold_method;
        rcfg.threshold_sensitivity = threshold_sensitivity;
        rcfg.fixed_amp_threshold = fixed_amp_threshold;
        rcfg.fixed_str_threshold = fixed_str_threshold;
        rcfg.signal_quality = signal_quality;
        rcfg.max_step_count = max_step_count;
        rcfg.reflector_shape = reflector_shape;
        rcfg.close_range_leakage = close_range_leakage;

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16);  // hexadecimal to unsigned integer
        QueueCmdRadarConfig( dest_id_dword, rcfg );

        return "Configure Radar command queued.";
    });

    // update sensor's info (version, name, uplink, etc..)
    CROW_ROUTE(gCrowApp,"/get-sensor-info/<string>")
    ([](std::string dest_id_str){
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdGetSensorInfo( dest_id_dword );

        return "Get Sensor Info command queued.";
    });

    // restart wake interval timer
    CROW_ROUTE(gCrowApp,"/restart-interval")
    ([](){
        RestartWakeInterval();

        return "Restarted Wake Interval Timer.";
    });

    // set tx/wake interval
    CROW_ROUTE(gCrowApp,"/set-interval/<uint>")
    ([](uint interval_sec ){

        QueueCmdSetInterval( interval_sec );

        return "Set Interval command queued.";
    });

    // set sensor's name
    CROW_ROUTE(gCrowApp,"/set-sensor-name/<string>/<string>")
    ([](std::string dest_id_str, std::string new_name ){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdSetSensorName( dest_id_dword, new_name );

        return "Set Sensor Name command queued.";
    });

    // set sensor's preferred uplink
    CROW_ROUTE(gCrowApp,"/set-preferred-uplink/<string>/<string>")
    ([](std::string dest_id_str, std::string uplink_id_str ){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        uint32_t uplink_id_dword = std::stoul(uplink_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdSetPreferredUplink( dest_id_dword, uplink_id_dword );

        return "Set Sensor Preferred Uplink command queued.";
    });

    // reboot the sensor
    CROW_ROUTE(gCrowApp,"/reset-sensor/<string>")
    ([](std::string dest_id_str){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdResetSensor( dest_id_dword );

        return "Reset Sensor command queued.";
    });

    // blink the sensors LEDs
    CROW_ROUTE(gCrowApp,"/blink-sensor/<string>")
    ([](std::string dest_id_str){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        QueueCmdBlinkLED( dest_id_dword );

        return "Blink Sensor LEDs command queued.";
    });

    // update sensor's firmware
    CROW_ROUTE(gCrowApp,"/update-sensorfw/<string>/<uint>")
    ([](std::string dest_id_str, uint32_t fw_idx){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        auto fw_list = GetFWList();
        std::filesystem::path fw_fname = fw_list.at(fw_idx).filename();

        // sanitize fw file path
        std::filesystem::path user_path;
        try {
            user_path = FIRMWARE_PATH / fw_fname;

            if (!std::filesystem::exists(user_path)){
                throw std::runtime_error("File does not exist.");
            }

            if (!std::filesystem::is_regular_file(user_path)){
                throw std::runtime_error("Not a regular file.");
            }

            // Prevent directory traversal
            if (!IsSubPath(FIRMWARE_PATH, user_path)){
                throw std::runtime_error("Access to this path is not allowed.");
            }
        }
        catch (const std::exception &ex){
            std::string err_msg = std::string("Error: ") + ex.what();
            std::cerr << err_msg << std::endl;
            return err_msg.c_str();
        }

        // begin the update procedure
        if ( !UpdateSensorFW( dest_id_dword, user_path ) ){
            return "UpdateSensorFW failed!";
        }

        return "Update Sensor FW started...";
    });

    // resume updating sensor's firmware
    CROW_ROUTE(gCrowApp,"/resume-sensorfw/<string>")
    ([](std::string dest_id_str){

        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        // resume the update procedure
        if ( !ResumeSensorFW( dest_id_dword ) ){
            return "ResumeSensorFW failed!";
        }

        return "Resuming Sensor FW update...";
    });

    // manually add a remote node to the server
    CROW_ROUTE(gCrowApp,"/manual-add-node/<string>")
    ([](std::string dest_id_str){
        uint32_t dest_id_dword = std::stoul(dest_id_str, nullptr, 16); // hexadecimal to unsigned integer
        if ( !ManuallyAddNode( dest_id_dword ) ){
            return fmt::format("Error: could not Manually add node {:08X}", dest_id_dword);
        }
        return fmt::format("Manually added node {:08X}", dest_id_dword);
    });

    gWebSvcFuture = gCrowApp.port(4202).run_async(); // MUST store the retval variable when calling run_async()
    gCrowApp.wait_for_server_start();

    return true; // success
}

bool IsWebServiceAlive( uint32_t timeout_ms ){
    auto rval = gWebSvcFuture.wait_for( std::chrono::milliseconds(timeout_ms) );
    return rval != std::future_status::ready;
}
