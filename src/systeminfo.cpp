#include "systeminfo.h"
#include <cmath>
#include <iostream>
#include <sys/sysinfo.h>
#include <stdio.h>
#include <stdlib.h>


std::string SystemInfo::getUpTimeFile()
{
    // old C style code for file and C++ for try/throw
    char buffer[128];

    FILE* pipe = popen("uptime", "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try
    {
        while (!feof(pipe))
        {
            if (fgets(buffer, 128, pipe) != NULL);
        }
    }
    catch (...)
    {
        pclose(pipe);
        throw;
    }
    pclose(pipe);

    std::string s(buffer);

 return s;
}

std::string SystemInfo::getUpTime()
{
    struct sysinfo sys_info;

    int days, hours, mins, x = 1;

    char buffer[128];

    if(sysinfo(&sys_info) != 0)
        perror("sysinfo");

    days  = sys_info.uptime / 86400;
    hours = (sys_info.uptime / 3600) - (days * 24);
    mins  = (sys_info.uptime / 60) - (days * 1440) - (hours * 60);

    sprintf(buffer,"%d:%d:%d:%ld",  days, hours, mins, sys_info.uptime % 60);

    std::string s(buffer);
    return s;
}


std::string SystemInfo::getSerialNumber()
{
    // old C style code for file and C++ for try/throw
    char buffer[128];
    // It shall create a pipe between the calling program and the executed command,
    // and shall return a pointer to a stream that can be used to either read from or write to the pipe.
    system("cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2 > ~/.serial");
    FILE* pipe = popen("cat ~/.serial", "r");
    // on error
    if (!pipe) throw std::runtime_error("popen() failed!");
    //reading shell stdout
    try
    {
        while (!feof(pipe))
        {
            if (fgets(buffer, 128, pipe) != NULL);
        }
    }
    catch (...)
    {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    // convert to string
    std::string s(buffer);

 return s;
}

std::string SystemInfo::getDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buffer[80];
    tstruct = *localtime(&now);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d.%X", &tstruct);
    std::string s(buffer);
    return s;
}


std::string SystemInfo::getLoadAverage(const int i)
{
    struct sysinfo sys_info;

    char buffer[128];

    if(sysinfo(&sys_info) != 0)
        perror("sysinfo");

    //Load Avgs: 1min(%ld) 5min(%ld) 15min(%ld)
    switch ( i )
      {
         case 1:
            sprintf(buffer,"%ld", sys_info.loads[0]);
            break;
         case 5:
            sprintf(buffer,"%ld", sys_info.loads[1]);
            break;
        case 15:
            sprintf(buffer,"%ld", sys_info.loads[2]);
            break;  
         default:
            sprintf(buffer,"%ld", sys_info.loads[0]);
      }

    std::string s(buffer);
    return s;
}

std::string SystemInfo::getFreeRam()
{
    struct sysinfo sys_info;

    char buffer[128];

    if(sysinfo(&sys_info) != 0)
        perror("sysinfo");

    sprintf(buffer,"%ld", sys_info.freeram / 1024);

    std::string s(buffer);
    return s;
}






