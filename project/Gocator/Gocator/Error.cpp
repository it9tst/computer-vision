#include "Error.h"

// constructors
GocatorCV::Error::Error(Error_Type type) {
    Type = type;
    SetMessage(type);
}

void GocatorCV::Error::DisplayMessage() { 
    std::cout << Message << std::endl;
}

void GocatorCV::Error::SetMessage(Error_Type type) {
    switch (type) {
        case OK:
            Message = "Entry cannot be empty!";
        case GENERIC:
            Message = "Entry contained invalid characters!";
            break;
        case GOSDK_API_CONSTRUCT:
            Message = "Error: GoSdk_Construct";
            break;
        case GOSYSTEM_CONSTRUCT:
            Message = "Error: GoSystem_Construct";
            break;
        case GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS:
            Message = "Error: GoSystem_FindSensorByIpAddress";
            break;
        case GOSENSOR_CONNECT:
            Message = "Error: GoSensor_Connect";
            break;
        case GOSENSOR_ENABLE_DATA: 
            Message = "Error: GoSensor_EnableData";
            break;
        case GOSYSTEM_START:
            Message = "Error: GoSystem_Start";
            break;
        case GOSYSTEM_STOP:
            Message = "Error: GoSystem_Stop";
            break;
    }
}