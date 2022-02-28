#include "Error.h"

// constructors
GocatorCV::Error::Error() {}

GocatorCV::Error::Error(Error_Type type, kStatus status) {
    type = type;
    status = status;
    GocatorCV::Error::SetMessage(type, status);
}

GocatorCV::Error_Type GocatorCV::Error::GetCode() {
    return this->type;
}

void GocatorCV::Error::DisplayMessage() { 
    std::cout << message << std::endl;
}

std::string GocatorCV::Error::GocatorCode(kStatus status) {
    switch (status) {
        case 1:
            return std::string("kOK");
        case 0:
            return std::string("kERROR");
        case -976:
            return std::string("kERROR_IN_PROGRESS");
        case -977:
            return std::string("kERROR_FULL");
        case -978:
            return std::string("kERROR_DEVICE");
        case -979:
            return std::string("kERROR_OS");
        case -980:
            return std::string("kERROR_CONFLICT");
        case -981:
            return std::string("kERROR_BUSY");
        case -982:
            return std::string("kERROR_WRITE_ONLY");
        case -983:
            return std::string("kERROR_READ_ONLY");
        case -984:
            return std::string("kERROR_FORMAT");
        case -985:
            return std::string("kERROR_HEAP");
        case -986:
            return std::string("kERROR_NETWORK");
        case -987:
            return std::string("kERROR_ALREADY_EXISTS");
        case -988:
            return std::string("kERROR_ABORT");
        case -989:
            return std::string("kERROR_VERSION");
        case -990:
            return std::string("kERROR_CLOSED");
        case -991:
            return std::string("kERROR_STREAM");
        case -992:
            return std::string("kERROR_INCOMPLETE");
        case -993:
            return std::string("kERROR_TIMEOUT");
        case -994:
            return std::string("kERROR_MEMORY");
        case -996:
            return std::string("kERROR_UNIMPLEMENTED");
        case -997:
            return std::string("kERROR_PARAMETER");
        case -998:
            return std::string("kERROR_COMMAND");
        case -999:
            return std::string("kERROR_NOT_FOUND");
        case -1000:
            return std::string("kERROR_STATE");
    }
}

void GocatorCV::Error::SetMessage(Error_Type type, kStatus status) {
    switch (type) {
        case OK:
            message = "Ok: " + GocatorCV::Error::GocatorCode(status);
        case GENERIC:
            message = "Error: Generic: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSDK_API_CONSTRUCT:
            message = "Error: GoSdk_Construct: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSYSTEM_CONSTRUCT:
            message = "Error: GoSystem_Construct: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS:
            message = "Error: GoSystem_FindSensorByIpAddress: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSENSOR_CONNECT:
            message = "Error: GoSensor_Connect: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSENSOR_SETUP:
            message = "Error: GoSensor_Setup: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSETUP_SETEXPOSURE:
            message = "Error: GoSetup_SetExposure: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSENSOR_ENABLE_DATA: 
            message = "Error: GoSensor_EnableData: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSYSTEM_START:
            message = "Error: GoSystem_Start: " + GocatorCV::Error::GocatorCode(status);
            break;
        case GOSYSTEM_STOP:
            message = "Error: GoSystem_Stop: " + GocatorCV::Error::GocatorCode(status);
            break;
    }
}