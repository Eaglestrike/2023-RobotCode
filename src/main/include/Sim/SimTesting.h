#include <string>
#include <units/time.h>
#include "Robot.h"

class SimTesting {
public:
    enum TestMode {TELEOPERATED, AUTONOMOUS, TEST};
    std::string runTest(std::string expectedFilePath, std::string logFileName, std::string debugFileName, units::time::second_t stepTime, TestMode mode);
private:
    void doTest();
    std::string analyze();
    std::string check();
    std::string generateDefaultFilePath(std::string filePath);
    std::vector<std::string> split(const std::string &s, char delim);

    std::optional<std::thread> m_thread;
    std::string expectedFilePath;
    std::string debugFileName;
    std::string logFileName;
    units::time::second_t stepTime;
    TestMode mode;
    std::vector<std::string> actualLogLines;
    std::vector<std::string> expectedLogLines;
    int randNum = 0;
};