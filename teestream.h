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

/// log to file and std::cerr
//
#include <iostream>
#include <fstream>
#include <filesystem>
#include <fmt/format.h>
#include <fmt/chrono.h>
#include <cstdarg>

// static std::string TSLogTimeStamp(){
//     char date[32];
//     time_t t = time(0);
//     tm my_tm;
//     localtime_r(&t, &my_tm);
//     size_t sz = strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &my_tm);
//     return std::string(date, date + sz);
// }
//
// // check streambuf for newline char
// class NewlineInterceptorBuffer : public std::streambuf {
// private:
//     std::streambuf* destBuffer;
//     bool lastWasNewline;
//
// public:
//     NewlineInterceptorBuffer(std::streambuf* dest)
//         : destBuffer(dest), lastWasNewline(false) {}
//
// protected:
//     virtual int_type overflow(int_type c) override {
//         if (c != traits_type::eof()) {
//             // If last character was newline and current isn't
//             if (lastWasNewline && c != '\n') {
//                 // Insert our custom string
//                 std::string ts_str = TSLogTimeStamp;
//                 destBuffer->sputn(ts_str.c_str(), ts_str.size());
//             }
//
//             // Write the character
//             destBuffer->sputc(c);
//
//             // Update newline tracking
//             lastWasNewline = (c == '\n');
//         }
//         return c;
//     }
//
//     virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
//         for (std::streamsize i = 0; i < n; ++i) {
//             overflow(s[i]);
//         }
//         return n;
//     }
// };

// log to file and std::cerr
class TeeStream {
public:
    TeeStream(std::ostream& stream1, const std::string &file_name)
        : LogFileName(file_name), LogFile(file_name,std::ios_base::app), stream1_(stream1) {
        if ( !LogFile ){
            std::cerr << "TeeStream log_file won't open." << std::endl;
        }
    }

    void CheckSize(){
        // if the logfile got too big, archive it
        LogFile << std::flush;
        std::filesystem::path lf_pathname( LogFileName );
        if ( std::filesystem::file_size( lf_pathname ) <= MAX_LOG_SIZE )
            return;

        *this << "Archiving log file\n";
        LogFile.close();
        // figure out the date today and create a new date stamped log file name
        std::time_t tnow = std::time(nullptr);
        std::tm *pnow = std::localtime(&tnow);
        int year = pnow->tm_year + 1900;
        int month = pnow->tm_mon + 1;
        int day = pnow->tm_mday;
        int hour = pnow->tm_hour;
        int minute = pnow->tm_min;

        std::string arc_filename = fmt::format( "{}-{:04d}-{:02d}-{:02d}_{:02d}-{:02d}{}",
            (std::string)lf_pathname.stem(), year, month, day, hour, minute,
            (std::string)lf_pathname.extension() );

        std::filesystem::path arc_pathname(lf_pathname);
        arc_pathname.replace_filename( arc_filename );

        // copy current log to archive
        std::cerr << "Copying log to " << arc_pathname << "\n";
        std::error_code ec;
        if ( !std::filesystem::copy_file(lf_pathname, arc_pathname, ec) ){
            std::cerr << "Could not rotate log file in CheckSize!\n";
        }

        // start new logfile
        std::filesystem::remove( lf_pathname );
        LogFile.open( lf_pathname, std::ios_base::app);
        if ( !LogFile ){
            std::cerr << "TeeStream log_file won't open.\n";
        }
    }

    template<typename T>
    TeeStream& operator<<(const T& value) {
        stream1_ << value;

        auto err_code = errno;
        if ( LogFile.fail() ){
            std::cerr << "TeeStream log_file fail. " << err_code << " - " << strerror(err_code) << std::endl;
        }
        else{
            LogFile << value;
        }
        return *this;
    }

    // Handle manipulators like endl
    TeeStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(stream1_);

        auto err_code = errno;
        if ( LogFile.fail() ){
            std::cerr << "TeeStream log_file fail. " << err_code << " - " << strerror(err_code) << std::endl;
        }
        else{
            manip(LogFile);
        }
        return *this;
    }

    TeeStream & Time(){
        // insert timestamp string
        *this << TimeStamp() << "  ";
        return *this;
    }

    // snprintf formatter
    TeeStream & Fmt( const char *fmt_str, ... ){
        char sbuf[1024];

        va_list args;
        va_start(args, fmt_str);
        int result = vsnprintf(sbuf, sizeof(sbuf), fmt_str, args);
        va_end(args);

        if ( result >= sizeof(sbuf) ){
            *this << "TeeStream::Fmt overrun!\n";
        }

        *this << sbuf;
        return *this;
    }

    static std::string TimeStamp(){
        // char date[32];
        // time_t t = time(0);
        // tm my_tm;
        // localtime_r(&t, &my_tm);
        // size_t sz = strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &my_tm);
        // return std::string(date, date + sz);
        // Get the current time_point
        auto now = std::chrono::system_clock::now();
        // Truncate to whole seconds and get the time_t
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
        // Format using fmt
        std::string formatted_time = fmt::format("{:%Y-%m-%d %H:%M:%S}.{:03}",
                                                *std::localtime(&now_time_t),
                                                ms.count());
        return formatted_time;
    }

private:
    const size_t MAX_LOG_SIZE = 1 * 1024 * 1024; // in bytes
    std::string LogFileName;;
    std::ofstream LogFile;
    std::ostream& stream1_;
};

extern TeeStream TSLogs;
