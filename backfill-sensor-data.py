# MIT License
#
# Copyright (c) 2026 John Andrew McInnes
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# A program to backfill sensor-data on cloud service
import csv
import json
import requests
import argparse
import sys
import datetime
import time

#todo:read sensor did -> cloud url mapping file
# SensorURLMap = {
#     "0A1928C9":"https://thingsboard.cloud/api/v1/fa3fasdgfasdfasfae34/telemetry",
#     "0A8654E2":"https://thingsboard.cloud/api/v1/tgsgz67fsgu73t6667ys/telemetry",
# }

def load_post_urls(posturls_filename) -> dict:
    """
    Load a 2-column CSV file into a dictionary, where each row becomes
    a key-value pair in the dictionary.

    :param csv_file: Path to the CSV file
    :return: A dictionary with keys from column 1 and values from column 2
    """
    result = {}
    with open(posturls_filename, 'r') as f:
        reader = csv.reader(f)
        #next(reader)  # Skip header row (if present)
        for row in reader:
            key, value = row[0], row[1]
            if key not in result:  # Avoid overwriting existing keys
                result[key] = value
    return result

def process_csv_to_cloud(csv_file_path, sensor_url_map:dict):
    # Hardcoded column headers for different telemetry types
    # original radar telemetry with 3 dist and 1 strength
    radar1_col_headers = [
        'timestamp', 'sentence', 'sensor_id', 'sensor_name', 'base_rssi', 'base_snr', 'base_freq_off',
        'wake_count', 'batt_voltage', 'last_rssi', 'last_snr', 'radar_dist0',
        'radar_strength0', 'radar_dist1', 'radar_dist2', 'temp_c', 'base_hops'
    ]

    # all 10 radar peaks distance and strength
    radar2_col_headers = [
        'timestamp', 'sentence', 'sensor_id', 'sensor_name', 'base_rssi', 'base_snr', 'base_freq_off',
        'wake_count', 'batt_voltage', 'last_rssi', 'last_snr', 'radar_dist0', 'radar_strength0',
        'radar_dist1', 'radar_strength1', 'radar_dist2', 'radar_strength2', 'radar_dist3', 'radar_strength3',
        'radar_dist4', 'radar_strength4', 'radar_dist5', 'radar_strength5','radar_dist6', 'radar_strength6',
        'radar_dist7', 'radar_strength7', 'radar_dist8', 'radar_strength8', 'radar_dist9', 'radar_strength9',
        'temp_c', 'base_hops'
    ]

    col_headers = radar1_col_headers

    # Open the CSV file for reading
    with open(csv_file_path, mode='r') as csv_file:
        # Create a CSV reader (not DictReader since we have our own col_headers)
        csv_reader = csv.reader(csv_file)

        # Process each row in the CSV
        for row in csv_reader:
            # Check if the row has enough columns and if sensor_id (index 1) is "radartlm"
            if len(row) > 1 and row[1] == 'radartlm':
                # calc proper timestamp
                try:
                    dt = datetime.datetime.fromisoformat(row[0])
                except Exception as e:
                    print(e)
                    continue
                epoch_ms = int(dt.timestamp() * 1000)

                # Create a dictionary by zipping col_headers with row values
                json_obj = {'ts':epoch_ms, "values":dict(zip(col_headers, row)) }

                #print('debug json')
                #print(json.dumps(json_obj, indent=4, sort_keys=False))

                # do HTTP POST
                sensor_id = row[2]
                if sensor_id in sensor_url_map:
                    url = sensor_url_map[sensor_id]
                    print( sensor_id + ' - ' + url, end='' )
                    http_headers = {"Content-Type": "application/json"}
                    # loop (re)trying post while it fails
                    num_tries = 100
                    while num_tries > 0:
                        num_tries -= 1
                        try:
                            response = requests.post(url, json=json_obj, headers=http_headers)
                            print(' - ' + str(response))
                            break
                        except requests.ConnectionError as e:
                            print(f' - {str(e)}')
                        except requests.Timeout as e:
                            print(f' - {str(e)}')
                        
                        print('Retrying...')
                        time.sleep(4)
    pass

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Process CSV file and convert radartlm entries to JSON.')
    parser.add_argument('input_files', nargs='+', type=str, help='input CSV filenames')

    # Parse arguments
    args = parser.parse_args()

    # Process the files
    surl_map = load_post_urls('posturls.csv')
    for filen in args.input_files:
        try:
       	    print('processing input file ' + filen)
            process_csv_to_cloud(filen, surl_map)
        except FileNotFoundError:
            print(f"Error: Input file not found: {filen}", file=sys.stderr)
            sys.exit(1)
        except Exception as e:
            print(f"Error processing file: {str(e)}", file=sys.stderr)
            sys.exit(1)

    print("done.")

# Example usage:
if __name__ == "__main__":
    main()
