#pragma once

//std c/c++
#include <iostream>
#include <string>

//GoSdk
#include <GoSdk/GoSdk.h>

namespace GocatorCV {

    enum ErrorType {
        GENERIC,
        OK,
        GOSDK_API_CONSTRUCT,
        GOSYSTEM_CONSTRUCT,
        GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS,
        GOSENSOR_CONNECT,
        GOSENSOR_MODEL,
        GOSENSOR_SETUP,
        GOSETUP_SETEXPOSURE,
        GOSENSOR_ENABLE_DATA,
        GOSYSTEM_START,
        GOSYSTEM_STOP
    };

    class Error {

    private:
        ErrorType type;
        kStatus status;
        std::string message;
        void SetMessage(ErrorType type, kStatus status);

    public:
        Error();
        Error(ErrorType type, kStatus status);
        ErrorType GetCode();
        std::string GocatorCode(kStatus status);
        std::string DisplayMessage();
    };
}