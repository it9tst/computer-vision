#pragma once

#include "Error.h"

//GoSdk
#include <GoSdk/GoSdk.h>

namespace GocatorCV{

    class Gocator {

    private:
        const char* sensor_ip;
        kStatus status;
        kAssembly api = kNULL;
        GoSystem system = kNULL;

    public:
        Gocator();
        Error Init();
        Error Start();
        Error Stop();
        void SetParameter(const char* sensor_ip);
        void Grab();
    };
}