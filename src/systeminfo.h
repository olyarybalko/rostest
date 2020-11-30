#ifndef SYSTEMINFO_H
#define SYSTEMINFO_H
#include <string>
class SystemInfo
{
    public:
        std::string getUpTime();
        std::string getSerialNumber();
        std::string getDateTime();
        std::string getFreeRam();
        std::string getUptimeSys();
        std::string getLoadAverage(const int i);
        std::string getRam();

};
#endif