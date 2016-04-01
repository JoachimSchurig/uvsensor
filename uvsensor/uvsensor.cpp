//
//  uvsensor.cpp
//
//  program to read the SI1145 sensor every second and writing averaged
//  values in a file and/or on stdout.
//
//  compile: g++ -std=gnu++14 -o uvsensor uvsensor.cpp Adafruit_SI1145.cpp -lcppgpio
//
//  call like: sudo ./uvsensor -i 60 -f uvsensor.data -p
//
//  Created by Joachim Schurig on 22.05.15, adapted to use CppGPIO 26.02.16
//
//  BSD license applies
//

#include "Adafruit_SI1145.h"

#include <stdlib.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cctype>
#include <chrono>


// prepare data types and storage for sensor values
struct value_t {
    uint16_t uv {0};
    uint32_t lux {0};
    uint32_t ir {0};
};

typedef std::vector<value_t> values_t;

struct parms_t {
    std::string filename;
    bool print_to_console {false};
    bool continuous {false};
    bool initialize {true};
    bool deinitialize {true};
    bool raw {false};
    bool query_settings {false};
    int interval {0};
    char single {0};
};



void read_sensor(const parms_t& parms)
{
    // create instance of sensor
    Adafruit_SI1145 sensor;
    
    if (parms.initialize) {
        // initialize sensor, deactivate proximity sensor and interrupts,
        // and set auto refresh to 0.5 seconds
        if (!sensor.begin(false, false, 500 * 1000)) {
            std::cerr << "Cannot init sensor" << std::endl;
            exit(1);
        }

        // sleep 5 ms to let the sensor fill with data before first read
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    // declare data storage
    values_t values;
    // initialize vector for easy iterator access
    // (+ 1 because 0 is a valid interval option)
    values.resize(parms.interval + 1);
    // and set iterator to the start of the vector
    values_t::iterator it = values.begin();
    
    // we read the sensor every second, and generate the average of
    // all values in the requested interval of 1..3600 seconds
    while (true) {
        
        if (parms.query_settings) {
            // query all current sample settings and print to stderr
            sensor.print_config();
            std::cerr << "\nVisRange: "         << sensor.readVisibleRange();
            std::cerr << "\nVisSensitivity: "   << sensor.readVisibleSensitivity();
            std::cerr << "\nIRRange: "          << sensor.readIRRange();
            std::cerr << "\nIRSensitivity: "    << sensor.readIRSensitivity();
            std::cerr << std::endl;
        }

        if (!parms.single || parms.single == 'U') it->uv  = sensor.readUV();
        if (!parms.single || parms.single == 'V') it->lux = (parms.raw) ? sensor.readVisible() : sensor.readVisibleLux();
        if (!parms.single || parms.single == 'I') it->ir  = (parms.raw) ? sensor.readIR() : sensor.readIRLux();

        // are we at the end of the values vector? Then prepare
        // output of the averaged values
        if (++it == values.end()) {
            
            uint32_t uv  = 0;
            // we do not need to use 64 bit values for the lux and ir accumulation,
            // although every single value is a 32 bit value itself
            // - the values rarely exceed 120k, and we sample at max 3600, so 32 bit
            // will suffice to add them!
            uint32_t lux = 0;
            uint32_t ir  = 0;
            
            // add all values up
            for (it = values.begin(); it != values.end(); ++it) {
                uv  += it->uv;
                lux += it->lux;
                ir  += it->ir;
            }

            // create average of all values
            uv  /= values.size();
            lux /= values.size();
            ir  /= values.size();
            
            // set iterator to start of vector for next round
            it = values.begin();
        
            if (!parms.filename.empty()) {
                
                std::ofstream out;
                out.open(parms.filename.c_str(), std::ofstream::out | std::ofstream::trunc);
                
                if (!out.is_open()) {
                    std::cerr << "Cannot open file " << parms.filename << std::endl;
                    exit(1);
                }
                
                out << std::fixed << std::setprecision(2);
                
                if (!parms.single || parms.single == 'U') out << uv / 100.0 << '\n';
                if (!parms.single || parms.single == 'V') out << lux << '\n';
                if (!parms.single || parms.single == 'I') out << ir << '\n';
                
                out.close();
                
            }
            
            if (parms.print_to_console) {
                
                std::cout << std::fixed << std::setprecision(2);
                
                if (!parms.single) {
                    
                    std::cout << "UV: " << uv / 100.0
                        << " LUX: " << lux
                        << " IR: " << ir << std::endl;
                    
                } else {
                    
                    switch (parms.single) {
                        case 'U':
                            std::cout << uv / 100.0 << std::endl;
                            break;
                        case 'V':
                            std::cout << lux << std::endl;
                            break;
                        case 'I':
                            std::cout << ir << std::endl;
                            break;
                    }
                    
                }
                
            }
            
            if (!parms.continuous) {
                
                if (parms.deinitialize) {
                    // switch sensor off
                    sensor.reset();
                }

                return;
                
            }
            
        }
        
        // sleep one second
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }
}


int main(int argc, char *argv[])
{
    // analyze options
    parms_t parms;

    {
        int opt;
        
        while ((opt = getopt(argc, argv, "cdf:hi:npqrs:")) != -1) {
            switch (opt) {
                case 'c':
                    parms.continuous = true;
                    break;
                case 'd':
                    parms.deinitialize = false;
                    break;
                case 'f':
                    parms.filename = optarg;
                    break;
                default:
                case 'h':
                    std::cout << "\n" << argv[0] << " - help:\n\n";
                    std::cout << " -c       : continuous mode\n";
                    std::cout << " -d       : do not deinit sensor, leave as is (implied by -n)\n";
                    std::cout << " -f file  : file to write sensor data into (default none)\n";
                    std::cout << " -i N     : sample interval in seconds to gather data (0..3600, default 0)\n";
                    std::cout << " -n       : do not init sensor, query as is\n";
                    std::cout << " -p       : print updates to stdout too (default off)\n";
                    std::cout << " -q       : query all sample settings\n";
                    std::cout << " -r       : query raw values\n";
                    std::cout << " -s [UVI] : query single value (either of UVI)\n\n";
                    exit(0);
                case 'i':
                    parms.interval = atoi(optarg);
                    if (parms.interval < 0 || parms.interval > 3600) {
                        std::cerr << "invalid value for interval (0..3600): " << parms.interval << std::endl;
                        exit(1);
                    }
                    break;
                case 'n':
                    parms.initialize = false;
                    parms.deinitialize = false;
                    break;
                case 'p':
                    parms.print_to_console = true;
                    break;
                case 'q':
                    parms.query_settings = true;
                    break;
                case 'r':
                    parms.raw = true;
                    break;
                case 's':
                    parms.single = toupper(optarg[0]);
                    switch (parms.single) {
                        case 'U':
                        case 'V':
                        case 'I':
                            break;
                        default:
                            std::cerr << "invalid value for single value (UVI): " << parms.single << std::endl;
                            exit(1);
                    }
                    break;
            }
        }
    }
    
    // and capture the sensor
    read_sensor(parms);
    
    return 0;
}
