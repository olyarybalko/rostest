#include "systeminfo.h"
#include <cmath>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>


std::string SystemInfo::getUpTime()
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


std::string SystemInfo::getSerialNumber()
{
    // old C style code for file and C++ for try/throw
    char buffer[128]={'0'};
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
    printf("%s\n", buffer);
    std::string s(buffer);

 return s;
}


