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

// code to forward sensor data to cloud services or web sites
// (C) 2025 John McInnes
//

#include <deque>
#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <httplib.h>
#include "lowhop.h"
#include "forwarddata.h"
//#include "teestream.h"

const char * POSTURL_FILENAME = "posturls.csv";

struct SensorDataPost_t{
    std::string web_url;
    std::string json_data;
};
static std::deque<SensorDataPost_t> gSDataPostQueue;    // queued HTTP post requests we need to execute
static std::mutex                   gQueueMutex;        // protects the queue
static std::condition_variable      gQueueCond;         // signals new data
static std::atomic<bool>            gWorkerRunning{true};// stop flag
static std::thread                  gWorkerThread;      // the worker itself

std::map<std::string, std::string> gWebSiteMap;
/* = {
    {"0A1928C9", "https://thingsboard.cloud/api/v1/2ohp4r3f4agrsdgadsas/telemetry"},
    {"0A8654E2", "https://thingsboard.cloud/api/v1/tgsgz67ga4hfdhaadfu7/telemetry"},
};
*/

// @brief load sensor id to post URL map from .csv file
void LoadSensorPostURLs(){
    std::ifstream inputFile( POSTURL_FILENAME );
    if (!inputFile) {
        TSLogs.Time() << "Error opening POST URL file.\n";
        return;
    }

    std::string line;
    while(getline(inputFile, line)){
        std::istringstream iss(line);
        std::string key, value;

        // Read the first column as the key
        getline(iss, key, ',');
        if (!key.size()) {
            TSLogs.Time() << "Invalid CSV format in " << POSTURL_FILENAME << "\n";
            return;
        }
        // Read the second column as the value
        getline(iss, value);

        gWebSiteMap[key] = value;
    }

    // for(auto &kv :gWebSiteMap){
    //     std::cout << kv.first << " " << kv.second << "\n";
    // }
}

// @brief queue telemetry for posting to cloud service
// pass json data string
void QueueSensorDataPost( const LHDeviceAddress_t &src_id, const std::string &sdata_str ){
    // lookup this sensor's POST url
    std::string thingsb_url = gWebSiteMap[src_id]; //"https://localhost:1231/api/v1/hm8tggic95ik2x16fylsp/telemetry";
    //std::cout << "websitemap: " << thingsb_url << std::endl;

    // queue it but don't wake worker yet'
    {
        std::lock_guard<std::mutex> lock_it(gQueueMutex);
        gSDataPostQueue.push_back( {thingsb_url, sdata_str} );
    }
}

void WakeSensorDataWorker(){
    // if not empty queue wake worker
    std::lock_guard<std::mutex> lock_it(gQueueMutex);
    if ( !gSDataPostQueue.empty() )
        gQueueCond.notify_one();    // wake the worker
}

/// @brief  Worker loop – runs in its own thread.
/// @details The function loops until `gWorkerRunning` becomes false
///          AND the queue is empty.
static void SensorDataPostWorker()
{
    // Regex is static so it is constructed only once
    const static std::regex url_re("^([^/]*/[^/]*/[^/]*)(/.*)$");
    std::smatch matches;

    while (true)
    {
        SensorDataPost_t sdata;
        // Grab a job from the queue (or exit if no more jobs & stopped)
        {
            std::unique_lock<std::mutex> lk(gQueueMutex);
            gQueueCond.wait(lk, [] {
                return !gSDataPostQueue.empty() || !gWorkerRunning.load();
            });

            // If we were woken because the worker is stopping and
            // the queue is empty – we are done.
            if (!gWorkerRunning && gSDataPostQueue.empty())
                break;

            // Pull the next job
            sdata = std::move(gSDataPostQueue.front());
            gSDataPostQueue.pop_front();
        }   // <-- unlocks the queue

        // process the job
        TSLogs.Time() << "Posting sensor data to: " << sdata.web_url << "\n";

        if (!std::regex_match(sdata.web_url, matches, url_re))
        {
            TSLogs.Time() << "Bad POST URL: " << sdata.web_url << "\n";
            continue;
        }

        const std::string web_site   = matches[1];
        const std::string web_endpt  = matches[2];

        // todo: watch for memory leak issue with SSL libs in thread: https://github.com/yhirose/cpp-httplib/issues/2144
        httplib::Client cli(web_site);
        cli.set_connection_timeout(5);
        cli.set_read_timeout(5, 0);
        cli.set_write_timeout(5, 0);

        auto res = cli.Post(web_endpt, sdata.json_data, "application/json" );
        if (res && res->status == 200) {
            std::cout << res->body << std::endl;
        }
        else{
            TSLogs.Time() << "Sensor Data Post error: " << res.error() << "\n";
        }
    }   // end while
}

/// @brief  Start the worker thread (idempotent).
void StartSensorDataPostWorker(){
    if (gWorkerThread.joinable())
        return;          // already running

    gWorkerRunning.store(true);
    gWorkerThread = std::thread(SensorDataPostWorker);
}

/// @brief  Signal the worker to finish and join the thread.
/// @details Must be called before program exit (or before the queue
///          goes out of scope) to avoid a dangling thread.
void StopSensorDataPostWorker(){
    gWorkerRunning.store(false);
    gQueueCond.notify_all();      // wake the worker if sleeping

    if (gWorkerThread.joinable())
        gWorkerThread.join();
}

// // @brief process the queued sensor data posts
// // @returns true on success
// bool DoSensorDataPosts(){
//     while( gSDataPostQueue.size() > 0 ){
//         SensorDataPost_t sdata = gSDataPostQueue.front();
//         gSDataPostQueue.pop_front();
//         TSLogs.Time() << "Posting sensor data to: " << sdata.web_url << "\n";
//
//         // split url into scheme-website-port and endpoint
//         //const static std::regex url_re(R"(^(?:(https?):)?(?://([^:/?#]*)(?::(\d+))?)?([^?#]*(?:\?[^#]*)?)(?:#.*)?)");
//         const static std::regex url_re("^([^/]*/[^/]*/[^/]*)(/.*)$");
//
//         std::smatch matches;
//         if ( !std::regex_match(sdata.web_url, matches, url_re) ){
//             TSLogs.Time() << "Bad POST URL: " << sdata.web_url << "\n";
//             continue;
//         }
//
//         //std::cout << web_site << " " << web_endpt << std::endl;
//         std::string web_site = matches[1];
//         std::string web_endpt = matches[2];
//
//         // do HTTP POST
//         httplib::Client cli(web_site);
//         cli.set_connection_timeout(5);
//         cli.set_read_timeout(5, 0); // 5 seconds
//         cli.set_write_timeout(5, 0);// 5 seconds
//
//         auto res = cli.Post(web_endpt, sdata.json_data, "application/json" );
//         if (res && res->status == 200) {
//             std::cout << res->body << std::endl;
//         }
//         else{
//             TSLogs.Time() << "Sensor Data Post error: " << res.error() << "\n";
//         }
//     }
//
//     return true;
// }
