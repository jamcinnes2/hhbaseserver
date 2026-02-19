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

/// templates/html/javascript for WebService
//

namespace WSTemplates{

    void GetHTML_NodeList( std::stringstream &page_text ){
        // interval clock
        std::time_t wake_time;
        uint32_t wake_sec;
        GetWakeInterval( wake_time, wake_sec );

        // manually add node button
        page_text << R"(
            <label for="manual_add_nodeid">Manually Add Sensor:</label>
            <input type="text" id="manual_add_nodeid" name="manual_add_nodeid" placeholder="ex. 0A112233" pattern="[A-Fa-f0-9]+">
            <button id="manual_add_nodeid_btn">add sensor</button>
        )";

        // node list/table div
        page_text << R"(
            <div id="node-list-id">
            <h3>network node list:</h3>
        )";
        page_text << "next wake time: <strong>" << CTimeToStr(wake_time) << "</strong> - interval: " << wake_sec << " seconds";
        page_text << R"(
            <table>
            <tr>
                <th>device ID</th>
                <th>protocol/fw/bl version</th>
                <th>name</th>
                <th>last_txtime</th>
                <th>wake_count</th>
                <th>base_hops</th>
                <th>preferred uplink</th>
                <th>cmdq-size</th>
                <th>dfu_path</th>
                <th>dfu_offset</th>
            </tr>
        )";

        auto nodes = GetLHNodesMetadata();
        for( const auto node : nodes ){
            std::string last_txtime_str = node.last_tx_time == 0 ? "0" : CTimeToStr(node.last_tx_time);
            page_text << "<tr>\n";
            page_text << "  <td>" << node.dev_id << "</td>\n";
            page_text << "  <td>" << node.protocol_ver << " / " << node.firmware_ver << " / " << node.bootloader_ver << "</td>\n";
            page_text << "  <td>" << node.dev_name << "</td>\n";
            page_text << "  <td>" << last_txtime_str << "</td>\n";
            page_text << "  <td>" << node.wake_count << "</td>\n";
            page_text << "  <td>" << node.base_hops << "</td>\n";
            page_text << "  <td>" << node.preferred_uplink_id << "</td>\n";
            page_text << "  <td>" << node.cmdq.size() << "</td>\n";
            page_text << "  <td>" << node.fw_path << "</td>\n";
            page_text << "  <td>" << node.fw_offset << "</td>\n";
            page_text << "</tr>\n";
        }
        page_text << "</table>";

        // node dropdown selector
        page_text << R"(
            <label for="node_select">Select Sensor:</label>
            <select id="node_select">
        )";
        for( const auto node : nodes ){
            page_text << "<option value=\"" << node.dev_id << "\">" << node.dev_id << " "
                << node.dev_name << "</option>";
        }
        page_text << R"(
            </select>
            <button id="download_sensor_data">download data</button>
            <br>
            <br>
            </div>
        )";
    }
    void GetJS_NodeList( std::stringstream &page_text ){
        page_text << R"(
            function apply_nodelist_listener(){
                // add download btn handler
                document.getElementById('download_sensor_data').addEventListener('click', function() {
                    var node_id_str = document.getElementById('node_select').value;
                    var log_dwnld_url = `/get-sensor-log/${node_id_str}`;
                    if ( window.download )
                        window.saveAs(log_dwnld_url);
                    else{
                        //window.open(log_dwnld_url, '_blank');
                        var link = document.createElement('a');
                        link.href = log_dwnld_url;
                        link.target = '_blank';
                        link.download = 'file.txt';
                        link.click();
                    }
                });

                // manual node add btn handler
                document.getElementById('manual_add_nodeid_btn').addEventListener('click', function() {
                    const node_id_input = document.getElementById('manual_add_nodeid');
                    var node_id_str = node_id_input.value.toUpperCase();
                    // validate node_id_str
                    const nid_regex = /^[0-9A-F]{8}$/;
                    if ( !nid_regex.test(node_id_str)){
                        alert("Invalid sensor ID. Please enter a hexadecimal number with exactly 8 characters.");
                        node_id_input.value = ""; // Clear the input field
                        return;
                    }
                    //
                    var add_node_url = `/manual-add-node/${node_id_str}`;
                    do_action_url( add_node_url );
                });
            }
            // apply on initial load
            apply_nodelist_listener();

            // Function to update the content of the div
            function update_nodelist() {
                // we have to fetch the entire page to get that div. (todo: optimize that?)
                fetch('/')
                    .then(response => response.text())
                    .then(html => {
                        const parser = new DOMParser();
                        const newDoc = parser.parseFromString(html, 'text/html');
                        var old_nodelist_div = document.getElementById('node-list-id');
                        var new_nodelist_div = newDoc.getElementById('node-list-id');

                        // Replace only the parts of the page that need updating
                        //      save and restore affected control states also
                        var node_sval = document.getElementById('node_select').value;

                        old_nodelist_div.innerHTML = new_nodelist_div.innerHTML;

                        document.getElementById('node_select').value = node_sval;

                        // reapply event listener to button
                        apply_nodelist_listener();
                    });
            }

            // Call the function every 20 seconds using setInterval()
            setInterval(update_nodelist, 20000);
        )";
    }

    void GetHTML_RadarConfig( std::stringstream &page_text ){
        // config radar form
        page_text << R"(
        <div class="container">
            <div id=config-radar-form class="left-div">
            <h3>Configure Radar</h3>
            <table>
            <tr>
                <td>
                <label for="xm125_distance_peak_sorting">Sort by:</label>
                </td>
                <td>
                <select id="xm125_distance_peak_sorting">
                    <option value="1">XM125_DISTANCE_CLOSEST</option>
                    <option value="2">XM125_DISTANCE_STRONGEST</option>
                </select>
                </td>
            </tr>
            <tr>
                <td>
                <label for="xm125_distance_profile">Max distance profile:</label>
                </td>
                <td>
                <select id="xm125_distance_profile">
                    <option value="1">XM125_DISTANCE_PROFILE1</option>
                    <option value="2">XM125_DISTANCE_PROFILE2</option>
                    <option value="3">XM125_DISTANCE_PROFILE3</option>
                    <option value="4">XM125_DISTANCE_PROFILE4</option>
                    <option value="5">XM125_DISTANCE_PROFILE5</option>
                </select>
                </td>
            </tr>
            <tr>
                <td>
                <label for="xm125_distance_threshold_method">Threshold method:</label>
                </td>
                <td>
                <select id="xm125_distance_threshold_method">
                    <option value="1">XM125_DISTANCE_FIXED_AMPLITUDE</option>
                    <option value="2">XM125_DISTANCE_RECORDED</option>
                    <option value="3">XM125_DISTANCE_CFAR</option>
                    <option value="4">XM125_DISTANCE_FIXED_STRENGTH</option>
                </select>
                </td>
            </tr>
            <tr>
                <td>
                <label for="xm125_distance_reflector_shape">Reflector shape:</label>
                </td>
                <td>
                <select id="xm125_distance_reflector_shape">
                    <option value="1">XM125_DISTANCE_GENERIC</option>
                    <option value="2">XM125_DISTANCE_PLANAR</option>
                </select>
                </td>
            </tr>
            <tr>
                <td>
                <label for="start_dist_mm">Start Distance (mm):</label>
                </td>
                <td>
                <input type="number" id="start_dist_mm" name="start_dist_mm" min="100" max="24000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="end_dist_mm">End Distance (mm):</label>
                <td>
                <input type="number" id="end_dist_mm" name="end_dist_mm" min="100" max="24000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="threshold_sensitivity">Threshold Sensitivity (0-1000):</label>
                </td>
                <td>
                <input type="number" id="threshold_sensitivity" name="threshold_sensitivity" min="0" max="1000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="fixed_amp_threshold">Fixed Amp Threshold (0-1000):</label>
                </td>
                <td>
                <input type="number" id="fixed_amp_threshold" name="fixed_amp_threshold" min="0" max="1000000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="fixed_str_threshold">Fixed Str Threshold (dBsm x1000):</label>
                </td>
                <td>
                <input type="number" id="fixed_str_threshold" name="fixed_str_threshold" min="-60000" max="60000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="signal_quality">Signal Quality (dB -10 to 35 x1000):</label>
                </td>
                <td>
                <input type="number" id="signal_quality" name="signal_quality" min="-10000" max="35000">
                </td>
            </tr>
            <tr>
                <td>
                <label for="max_step_count">Max Step Count (default 0):</label>
                </td>
                <td>
                <input type="number" id="max_step_count" name="max_step_count" min="0" max="1000">
                </td>
            </tr>
            <tr>
                <td>Close Range Leakage:</td>
                <td><input type="checkbox" id="close_range_leakage" name="close_range_leakage"> Yes</td>
            </tr>
            </table>
            <br>
            <button id="cfg_radar_btn">send radar cfg</button>
            <button id="get_radarcfg_btn">request radar cfg</button>
            <button id="show_radarcfg_btn">show received radar cfg</button>
        )";
    }

    void GetJS_RadarConfig( std::stringstream &page_text ){
        // script for config radar form
        page_text << R"(
            function get_configradar_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var start_dist_mm   = document.getElementById('start_dist_mm').value;
                var end_dist_mm     = document.getElementById('end_dist_mm').value;
                var max_profile     = document.getElementById('xm125_distance_profile').value;
                var peak_sorting        = document.getElementById('xm125_distance_peak_sorting').value;
                var threshold_method    = document.getElementById('xm125_distance_threshold_method').value;
                var threshold_sensitivity   = document.getElementById('threshold_sensitivity').value;
                var fixed_amp_threshold = document.getElementById('fixed_amp_threshold').value;
                var fixed_str_threshold = document.getElementById('fixed_str_threshold').value;
                var signal_quality  = document.getElementById('signal_quality').value;
                var max_step_count  = document.getElementById('max_step_count').value;
                var reflector_shape = document.getElementById('xm125_distance_reflector_shape').value;
                var close_range_leakage = document.getElementById('close_range_leakage').checked ? 1 : 0;

                var url_str = `/config-radar/${node_id_str}/${start_dist_mm}/${end_dist_mm}/${max_profile}`;
                url_str += `/${peak_sorting}/${threshold_method}/${threshold_sensitivity}/${fixed_amp_threshold}`;
                url_str += `/${fixed_str_threshold}/${signal_quality}/${max_step_count}/${reflector_shape}`;
                url_str += `/${close_range_leakage}`;
                return url_str;
            }

            // do form actions
            document.getElementById('cfg_radar_btn').addEventListener('click', function() {
                do_action_url( get_configradar_url() );
            });

            document.getElementById('get_radarcfg_btn').addEventListener('click', function() {
                // get radar config
                var node_id_str      = document.getElementById('node_select').value;
                var get_radarcfg_url = `/get-radar-config/${node_id_str}`;

                do_action_url( get_radarcfg_url );
            });

            // get Last Received radar config
            function get_last_configradar(){
                var node_id_str = document.getElementById('node_select').value;
                var getlast_radarcfg_url = `/get-last-radar-config/${node_id_str}`;

                fetch(getlast_radarcfg_url)
                    .then(response => response.json())
                    .then(data => {
                        console.log(data);
                        if ( data['start_dist_mm'] == -1 ){
                            document.getElementById("result").innerHTML = "Request failed - invalid radar configuration (did we receive one yet?) ";
                            return; // invalid rcfg
                        }

                        document.getElementById('xm125_distance_profile').value = data['max_profile'];
                        document.getElementById('xm125_distance_peak_sorting').value = data['peak_sorting'];
                        document.getElementById('xm125_distance_threshold_method').value = data['threshold_method'];
                        document.getElementById('xm125_distance_reflector_shape').value = data['reflector_shape'];
                        document.getElementById('start_dist_mm').value = data['start_dist_mm'];
                        document.getElementById('end_dist_mm').value = data['end_dist_mm'];
                        document.getElementById('threshold_sensitivity').value = data['threshold_sensitivity'];
                        document.getElementById('fixed_amp_threshold').value = data['fixed_amp_threshold'];
                        document.getElementById('fixed_str_threshold').value = data['fixed_str_threshold'];
                        document.getElementById('signal_quality').value = data['signal_quality'];
                        document.getElementById('max_step_count').value = data['max_step_count'];
                        document.getElementById('close_range_leakage').checked = value;
                    })
                    .catch(error => console.error('Error: ', error));
            }

            document.getElementById('show_radarcfg_btn').addEventListener('click', function() {
                get_last_configradar();
            });
        )";
    }

    void GetHTML_GetSensorInfo( std::stringstream &page_text ){
        page_text << R"(
            <h3>Get Sensor Info</h3>
            <div id=get-sensor-info>
            <button id="get_sensor_info_btn">Get Sensor Info</button>
            </div>
          </div>
          <div class="right-div">
        )";
    }

    void GetJS_GetSensorInfo( std::stringstream &page_text ){
        page_text << R"(
            function get_getsensorinfo_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var url_str = `/get-sensor-info/${node_id_str}`;
                return url_str;
            }

            // do form actions
            document.getElementById('get_sensor_info_btn').addEventListener('click', function() {
                do_action_url( get_getsensorinfo_url() );
            });
        )";
    }

    void GetHTML_SetInterval( std::stringstream &page_text ){
        page_text << R"(
            <h3>Set Telemetry Interval</h3>
            <div id=set-interval>
            <label for="tlm_interval">Telemetry Sending Interval (sec):</label>
            <input type="number" id="tlm_interval" name="tlm_interval" min="30" max="21600">
            <button id="set_interval_btn">set interval</button>
            <button id="restart_interval_btn">restart interval</button>
            </div>
        )";
    }

    void GetJS_SetInterval( std::stringstream &page_text ){
        page_text << R"(
            function get_setinterval_url(){
                var tlm_interval_sec = document.getElementById('tlm_interval').value;

                var url_str = `/set-interval/${tlm_interval_sec}`;
                return url_str;
            }

            function get_restartinterval_url(){
                var tlm_interval_sec = document.getElementById('tlm_interval').value;

                var url_str = `/restart-interval`;
                return url_str;
            }

            // do form actions
            document.getElementById('set_interval_btn').addEventListener('click', function() {
                do_action_url( get_setinterval_url() );
            });
            document.getElementById('restart_interval_btn').addEventListener('click', function() {
                do_action_url( get_restartinterval_url() );
            });
        )";
    }

    void GetHTML_SetSensorName( std::stringstream &page_text ){
        page_text << R"(
            <h3>Set Sensor Name</h3>
            <div id=set-sensor-name>
            <label for="sensor_name">Sensor name (12 chars max):</label>
            <input type="text" id="sensor_name" name="sensor_name" >
            <button id="set_sensor_name_btn">set name</button>
            </div>
        )";
    }

    void GetJS_SetSensorName( std::stringstream &page_text ){
        page_text << R"(
            function get_setsensorname_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var sensor_name_str = document.getElementById('sensor_name').value;

                var url_str = `/set-sensor-name/${node_id_str}/${sensor_name_str}`;
                return url_str;
            }

            // do form actions
            document.getElementById('set_sensor_name_btn').addEventListener('click', function() {
                do_action_url( get_setsensorname_url() );
            });
        )";
    }

    void GetHTML_SetPreferredUplink( std::stringstream &page_text ){
        page_text << R"(
            <h3>Set Preferred Uplink</h3>
            <div id=set-preferred-uplink>
            <label for="preferred_uplink">Preferred Uplink:</label>
            <input type="text" id="preferred_uplink" name="preferred_uplink" >
            <button id="set_preferred_uplink_btn">set uplink</button>
            </div>
        )";
    }

    void GetJS_SetPreferredUplink( std::stringstream &page_text ){
        page_text << R"(
            function get_setpreferreduplink_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var uplink_id_str   = document.getElementById('preferred_uplink').value;

                var url_str = `/set-preferred-uplink/${node_id_str}/${uplink_id_str}`;
                return url_str;
            }

            // do form actions
            document.getElementById('set_preferred_uplink_btn').addEventListener('click', function() {
                do_action_url( get_setpreferreduplink_url() );
            });
        )";
    }

    void GetHTML_BlinkLED( std::stringstream &page_text ){
        page_text << R"(
            <h3>Blink LEDs</h3>
            <div id=blink-led>
            <button id="blink_led_btn">Blink LEDs</button>
            </div>
        )";
    }

    void GetJS_BlinkLED( std::stringstream &page_text ){
        page_text << R"(
            function get_blinkled_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var url_str = `/blink-sensor/${node_id_str}`;
                return url_str;
            }

            // do form actions
            document.getElementById('blink_led_btn').addEventListener('click', function() {
                do_action_url( get_blinkled_url() );
            });
        )";
    }

    void GetHTML_ResetSensor( std::stringstream &page_text ){
        page_text << R"(
            <h3>Reboot Sensor</h3>
            <div id=reset-sensor>
            <button id="reset_sensor_btn">Reset Sensor</button>
            <br><br>for debugging
            <button id="pkt_now_btn">Send Get Version Now!</button>
            <button id="recalc_toffsets_btn">Recalc Telemetry Offsets</button>
            </div>
        )";
    }

    void GetJS_ResetSensor( std::stringstream &page_text ){
        page_text << R"(
            function get_resetsensor_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var url_str = `/reset-sensor/${node_id_str}`;
                return url_str;
            }

            function get_pktnow_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var url_str = `/send-pktnow/${node_id_str}`;
                return url_str;
            }

            function get_recalctoffsets_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var url_str = `/recalc-toffsets`;
                return url_str;
            }

            // do form actions
            document.getElementById('reset_sensor_btn').addEventListener('click', function() {
                do_action_url( get_resetsensor_url() );
            });
            document.getElementById('pkt_now_btn').addEventListener('click', function() {
                do_action_url( get_pktnow_url() );
            });
            document.getElementById('recalc_toffsets_btn').addEventListener('click', function() {
                do_action_url( get_recalctoffsets_url() );
            });
        )";
    }

    // update sensor firmware
    void GetHTML_UpdateSensorFW( std::stringstream &page_text ){
        auto fw_list = GetFWList();
        page_text << R"(
            <h3>Update Firmware</h3>
            <div id=update-sensorfw>
            <label for="fw_select">FW Name:</label>
            <select id="fw_select">
            )";
            int idx = 0;
            for( const auto fw_path : fw_list ){
                page_text << "<option value=\"" << idx++ << "\">" << fw_path.filename().string() << "</option>\n";
            }
        page_text <<
        R"(</select>
            &nbsp;&nbsp;
            <button id="update_sensorfw_btn">Update Sensor Firmware</button>
            <button id="resume_sensorfw_btn">Resume Update Sensor FW</button>
            </div>
        )";
    }

    void GetJS_UpdateSensorFW( std::stringstream &page_text ){
        page_text << R"(
            function get_updatesensorfw_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var fw_name_str     = document.getElementById('fw_select').value;
                var url_str = `/update-sensorfw/${node_id_str}/${fw_name_str}`;
                return url_str;
            }

            function get_resumesensorfw_url(){
                var node_id_str     = document.getElementById('node_select').value;
                var fw_name_str     = document.getElementById('fw_select').value;
                var url_str = `/resume-sensorfw/${node_id_str}`;
                return url_str;
            }

            // do form actions
            document.getElementById('update_sensorfw_btn').addEventListener('click', function() {
                do_action_url( get_updatesensorfw_url() );
            });
            document.getElementById('resume_sensorfw_btn').addEventListener('click', function() {
                do_action_url( get_resumesensorfw_url() );
            });
        )";
    }

    // tail the server log
    void GetHTML_BaseServerLog( std::stringstream &page_text ){
        //<div id="server-log" class="scrollable-box">
        page_text << R"(
            <div align="center">
              <strong>Server Log</strong>
              <br>
              <br>
              <textarea id="server-log" readonly>loading server log...</textarea>
            </div>
        )";
    }

    void GetJS_BaseServerLog( std::stringstream &page_text ){
        page_text << R"(
            // Function to update the content of the div
            function update_serverlog() {
                var url_str = "/server-log";
                update_log_ctrl(url_str);
            }

            // Call the function every 10 seconds using setInterval()
            setInterval(update_serverlog, 10000); // 10 seconds in milliseconds
        )";
    }

} // end namespace
