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

/// radar sensor net base server app
//

#include "lowhop.h"
#include "loracomms.h"
#include "webservice.h"
#include "forwarddata.h"
#include "timer.h"

#define POST_AFTER_RXWINDOW_SEC 5   // Post cloud data AFTER RX window interval. Trying to avoid interference
                                    // ..between the wifi/cellular radio and the LoRa radios.

// our logger
TeeStream TSLogs(std::cout, "base_server.log");

// the entry point for the program
int main(int argc, char** argv) {
    setlinebuf(stdout); // solves issue incase someone pipes to Linux tee
    TSLogs.CheckSize(); // rotate log file if needed
    TSLogs.Time() << "Sensor Base Server v" << BASE_FIRMWARE_VERSION_STR << " starting...\n";

    InitLoRa();

    if ( !StartWebService() ){
        TSLogs.Time() << "StartWebService failed" << std::endl;
        exit(-1);
        return(-1);
    }

    LoadSensorPostURLs();
    StartSensorDataPostWorker();

    SimpleTimer log_rotate;
    log_rotate.start();

    // loop while webapp runs, polling lora. sleep 10ms
    while( IsWebServiceAlive(10) ){
        ProcessLoRa();

        // if radio isn't busy (if we are not in RX Window), post data.
        // or if the rx window is too short to wait around, post data.
        if ( GetSecPastRXWindow() >= POST_AFTER_RXWINDOW_SEC ||
             GetLHWakeInterval() <= LOWHOP_RX_WINDOW_SEC+POST_AFTER_RXWINDOW_SEC ){
            WakeSensorDataWorker();
        }

        // check on log file once a day
        if ( log_rotate.elapsed_sec() > (24 * 60 * 60) ){
            log_rotate.restart();
            TSLogs.CheckSize();
        }
    }

    return(0);
}
