#pragma once

//std c/c++
#include <iostream>

namespace GocatorCV {

    enum Error_Type {
        OK = 0,
        GENERIC,
        GOSDK_API_CONSTRUCT,
        GOSYSTEM_CONSTRUCT,
        GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS,
        GOSENSOR_CONNECT,
        GOSENSOR_ENABLE_DATA,
        GOSYSTEM_START,
        GOSYSTEM_STOP
    };

    class Error {

    private:
        Error_Type Type;
        std::string Message;
        void SetMessage(Error_Type type);

    public:
        Error(Error_Type type);
        void DisplayMessage();
    };
}