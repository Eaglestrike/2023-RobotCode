#include "SimTesting.h"
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogReader.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <fstream>


std::string SimTesting::runTest(std::string expectedFilePath, std::string logFileName, std::string debugFileName, units::time::second_t stepTime, TestMode mode) {
    //frc::DataLogManager::GetLog().SetFilename("HHIIIi");
    this->expectedFilePath = expectedFilePath;
    this->debugFileName = generateDefaultFilePath(debugFileName) + ".csv";
    this->logFileName = generateDefaultFilePath(logFileName) + ".wpilog";
    this->stepTime = stepTime;
    this->mode = mode;
    doTest();
    sleep(3); // make sure log file is flushed to disk
    std::string ret = analyze();
    if (ret != "") {
        return ret;
    }
    return check();
}

void SimTesting::doTest() {
    std::cout << "Running tests" << std::endl;

    frc::sim::StepTiming(0.0_ms);  // Wait for Notifiers

    if (mode == TELEOPERATED) {
        frc::sim::DriverStationSim::SetTest(false);
        frc::sim::DriverStationSim::SetAutonomous(false);
    } else if (mode == AUTONOMOUS) {
        frc::sim::DriverStationSim::SetTest(false);
        frc::sim::DriverStationSim::SetAutonomous(true);
    } else if (mode == TEST) {
        frc::sim::DriverStationSim::SetTest(true);
    }

    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();

    frc::DataLogManager::LogNetworkTables(false);
    frc::DataLogManager::Start();
    frc::DataLogManager::GetLog().SetFilename(logFileName);

    frc::sim::StepTiming(stepTime);
    
    frc::DataLogManager::GetLog().Flush();

    std::cout << "Tests run" << std::endl;
}

std::string SimTesting::analyze() {
    std::cout << "Analyzing tests" << std::endl;

    std::error_code ec;
    wpi::log::DataLogReader reader{wpi::MemoryBuffer::GetFile(logFileName, ec)};
    if (ec) {
        return "could not open file: {}\n" + ec.message();
    }
    if (!reader) {
        return "not a log file\n";
    }

    std::ifstream expectedLog(expectedFilePath);
    if (!expectedLog.is_open()) {
        return "could not open expected log file\n";
    }

    std::string expectedLine;

    if (!getline(expectedLog, expectedLine)) {
        expectedLog.close();
        return "expected log file too short\n";
    }
    std::vector<std::string> watchEntries = split(expectedLine, ',');
    expectedLogLines = std::vector<std::string>();
    while (std::getline(expectedLog, expectedLine)) {
        expectedLogLines.push_back(expectedLine);
    }
    expectedLog.close();

    actualLogLines = std::vector<std::string>();

    std::ofstream outfile;
    outfile.open(debugFileName);
    if (!outfile.is_open()) {
        return "could not open debug log file\n";
    }
    
    int64_t startTime = 0;
    wpi::DenseMap<int, wpi::log::StartRecordData> entries;
    for (auto&& record : reader) {
        if (record.IsStart()) {
            wpi::log::StartRecordData data;
            if (record.GetStartData(&data)) {
                fmt::print("Start({}, name='{}', type='{}', metadata='{}') [{}]\n",
                        data.entry, data.name, data.type, data.metadata,
                        record.GetTimestamp() / 1000000.0);
                if (entries.find(data.entry) != entries.end()) {
                    fmt::print("...DUPLICATE entry ID, overriding\n");
                }
                entries[data.entry] = data;
            } else {
                fmt::print("Start(INVALID)\n");
            }
        } else if (record.IsFinish()) {
            int entry;
            if (record.GetFinishEntry(&entry)) {
                fmt::print("Finish({}) [{}]\n", entry, record.GetTimestamp() / 1000000.0);
                auto it = entries.find(entry);
                if (it == entries.end()) {
                    fmt::print("...ID not found\n");
                } else {
                    entries.erase(it);
                }
            } else {
                fmt::print("Finish(INVALID)\n");
            }
        } else if (record.IsSetMetadata()) {
            wpi::log::MetadataRecordData data;
            if (record.GetSetMetadataData(&data)) {
                fmt::print("SetMetadata({}, '{}') [{}]\n", data.entry, data.metadata,
                    record.GetTimestamp() / 1000000.0);
                auto it = entries.find(data.entry);
                if (it == entries.end()) {
                    fmt::print("...ID not found\n");
                } else {
                    it->second.metadata = data.metadata;
                }
            } else {
                fmt::print("SetMetadata(INVALID)\n");
            }
        } else if (record.IsControl()) {
            //fmt::print("Unrecognized control record\n");
        } else {
            fmt::print("Data({}, size={}) ", record.GetEntry(), record.GetSize());
            auto entry = entries.find(record.GetEntry());
            if (entry == entries.end()) {
                fmt::print("<ID not found>\n");
                continue;
            }
            fmt::print("<name='{}', type='{}'> [{}]\n", entry->second.name,
                        entry->second.type, record.GetTimestamp() / 1000000.0);

            // handle systemTime specially
            if (entry->second.name == "systemTime" && entry->second.type == "int64") {
                int64_t val;
                if (record.GetInteger(&val)) {
                    // Below doesn't work due to compiler error, skipping for now
                    //std::time_t timeval = val / 1000000;
                    //fmt::print("  {:%Y-%m-%d %H:%M:%S}.{:06}\n", *std::localtime(&timeval), val % 1000000);
                } else {
                    fmt::print("  invalid\n");
                }
                continue;
            }

            if (entry->second.type == "double") {
                double val;
                if (record.GetDouble(&val)) {
                    fmt::print("  {}\n", val);
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, val);
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "int64") {
                int64_t val;
                if (record.GetInteger(&val)) {
                    fmt::print("  {}\n", val);
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, val);
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "string" ||
                        entry->second.type == "json") {
                std::string_view val;
                if (record.GetString(&val)) {
                    fmt::print("  '{}'\n", val);
                    if (entry->second.name == "/simtesting/loginfo" && val == "START") {
                        startTime = record.GetTimestamp();
                    } else {
                        double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                        std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, val);
                        if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                            actualLogLines.push_back(line);
                        }
                        outfile << line << std::endl;
                    }
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "boolean") {
                bool val;
                if (record.GetBoolean(&val)) {
                    fmt::print("  {}\n", val);
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, val);
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "boolean[]") {
                std::vector<int> val;
                if (record.GetBooleanArray(&val)) {
                    fmt::print("  {}\n", fmt::join(val, ", "));
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, fmt::join(val, "|"));
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "double[]") {
                std::vector<double> val;
                if (record.GetDoubleArray(&val)) {
                    fmt::print("  {}\n", fmt::join(val, ", "));
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, fmt::join(val, "|"));
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "float[]") {
                std::vector<float> val;
                if (record.GetFloatArray(&val)) {
                    fmt::print("  {}\n", fmt::join(val, ", "));
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, fmt::join(val, "|"));
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "int64[]") {
                std::vector<int64_t> val;
                if (record.GetIntegerArray(&val)) {
                    fmt::print("  {}\n", fmt::join(val, ", "));
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, fmt::join(val, "|"));
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            } else if (entry->second.type == "string[]") {
                std::vector<std::string_view> val;
                if (record.GetStringArray(&val)) {
                    fmt::print("  {}\n", fmt::join(val, ", "));
                    double rawTime = (record.GetTimestamp() - startTime) / 1000000.0;
                    std::string line = fmt::format("{:.2f},{},{}", rawTime, entry->second.name, fmt::join(val, "|"));
                    if (std::find(watchEntries.begin(), watchEntries.end(), entry->second.name) != watchEntries.end()) {
                        actualLogLines.push_back(line);
                    }
                    outfile << line << std::endl;
                } else {
                    fmt::print("  invalid\n");
                }
            }
        }
    }

    outfile.close();

    std::cout << "Test successfully analyzed" << std::endl;
    return "";
}

std::string SimTesting::check() {
    std::cout << "Checking log files" << std::endl;
    for (uint i = 0; i < actualLogLines.size() && i < expectedLogLines.size(); i++) {
        if (actualLogLines[i] != expectedLogLines[i]) {
            std::string ret = "Actual log file does not match expected log file\nActual log file line " 
                + std::to_string(i) + ": \"" + actualLogLines[i] + "\"\n" +
                "Expected log file line " + std::to_string(i) + ": \"" + expectedLogLines[i] + "\"\n";
            return ret;
        }
    }
    if (actualLogLines.size() < expectedLogLines.size()) {
        return "Lines match in log files, actual log file is shorter than expected log file";
    } else if (actualLogLines.size() > expectedLogLines.size()) {
        return "Lines match in log files, actual log file is longer than expected log file";
    }
    std::cout << "OK Log files match" << std::endl;

    return "";
}

std::string SimTesting::generateDefaultFilePath(std::string filePath) {
    if (randNum == 0) {
        std::srand(std::time(nullptr));
        randNum = std::rand() % 1000;
    }

    auto timestamp = std::chrono::system_clock::now();
    std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tm = *std::localtime(&now_tt);
    std::ostringstream oss;
    auto time = std::put_time(&tm, "%c %Z");
    oss << time;
    std::string timeStr = oss.str();

    return filePath + "-" + timeStr + "-" + std::to_string(randNum);
}

// Source: https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> SimTesting::split(const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}