#pragma once

//std c/c++
#include <iostream>
#include <string>

//GoSdk
#include <GoSdk/GoSdk.h>

namespace GocatorCV {

    enum Error_Type {
        GENERIC = 0,
        OK,
        GOSDK_API_CONSTRUCT,
        GOSYSTEM_CONSTRUCT,
        GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS,
        GOSENSOR_CONNECT,
        GOSENSOR_SETUP,
        GOSETUP_SETEXPOSURE,
        GOSENSOR_ENABLE_DATA,
        GOSYSTEM_START,
        GOSYSTEM_STOP
    };

    class Error {

    private:
        Error_Type type;
        kStatus status;
        std::string message;
        void SetMessage(Error_Type type, kStatus status);

    public:
        Error();
        Error(Error_Type type, kStatus status);
        Error_Type GetCode();
        std::string GocatorCode(kStatus status);
        void DisplayMessage();
    };
}