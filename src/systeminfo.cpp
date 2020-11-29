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
