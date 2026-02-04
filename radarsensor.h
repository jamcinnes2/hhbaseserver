// MIT License
//
// Copyright (c) 2024 John Andrew McInnes
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

// XM125 Radar sensor functions
//
//#include "SparkFun_Qwiic_XM125_Arduino_Library.h"  // JAM: I made a change to fix bugs

typedef enum
{
    XM125_DISTANCE_CLOSEST = 1,
    XM125_DISTANCE_STRONGEST = 2,
} xm125_distance_peak_sorting_t;

typedef enum
{
    XM125_DISTANCE_PROFILE1 = 1,
    XM125_DISTANCE_PROFILE2 = 2,
    XM125_DISTANCE_PROFILE3 = 3,
    XM125_DISTANCE_PROFILE4 = 4,
    XM125_DISTANCE_PROFILE5 = 5,
} xm125_distance_profile_t;

typedef enum
{
    XM125_DISTANCE_FIXED_AMPLITUDE = 1,
    XM125_DISTANCE_RECORDED = 2,
    XM125_DISTANCE_CFAR = 3,
    XM125_DISTANCE_FIXED_STRENGTH = 4,
} xm125_distance_threshold_method_t;

typedef enum
{
    XM125_DISTANCE_GENERIC = 1,
    XM125_DISTANCE_PLANAR = 2,
} xm125_distance_reflector_shape_t;

#define DEFAULT_RADAR_START_MM 500
#define DEFAULT_RADAR_END_MM 12000
#define DEFAULT_RADAR_PEAK_SORTING XM125_DISTANCE_STRONGEST
#define DEFAULT_RADAR_MAX_STEP_COUNT 0  // 0 means let profile choose
#define DEFAULT_RADAR_MAX_PROFILE XM125_DISTANCE_PROFILE1
#define DEFAULT_RADAR_THRESH_METHOD XM125_DISTANCE_FIXED_STRENGTH
#define DEFAULT_RADAR_THRESH_SENSITIVITY 500
#define DEFAULT_RADAR_FIXED_AMP_THRESH 20
#define DEFAULT_RADAR_FIXED_STR_THRESH 0
#define DEFAULT_RADAR_SIGNAL_QUALITY 20000
#define DEFAULT_RADAR_REFLECTOR_SHAPE XM125_DISTANCE_GENERIC
#define DEFAULT_RADAR_CLOSE_RANGE_LEAK 0

struct RadarConfiguration_t {
    int32_t start_dist_mm = DEFAULT_RADAR_START_MM;
    int32_t end_dist_mm = DEFAULT_RADAR_END_MM;
    int32_t max_profile = DEFAULT_RADAR_MAX_PROFILE;
    int32_t peak_sorting = DEFAULT_RADAR_PEAK_SORTING;
    int32_t threshold_method = DEFAULT_RADAR_THRESH_METHOD;
    int32_t threshold_sensitivity = DEFAULT_RADAR_THRESH_SENSITIVITY;
    int32_t fixed_amp_threshold = DEFAULT_RADAR_FIXED_AMP_THRESH;
    int32_t fixed_str_threshold = DEFAULT_RADAR_FIXED_STR_THRESH;
    int32_t signal_quality = DEFAULT_RADAR_SIGNAL_QUALITY;
    int32_t max_step_count = DEFAULT_RADAR_MAX_STEP_COUNT;
    int32_t reflector_shape = DEFAULT_RADAR_REFLECTOR_SHAPE;
    uint8_t close_range_leakage = DEFAULT_RADAR_CLOSE_RANGE_LEAK;
};


