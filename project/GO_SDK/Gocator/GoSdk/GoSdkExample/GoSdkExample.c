#include <GoSdk/GoSdk.h>

kStatus ExampleMain()
{
    const k32u sensorId = 000000;
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;

    kCheck(GoSdk_Construct(&api));

    kCheck(GoSystem_Construct(&system, kNULL));

    kCheck(GoSystem_FindSensorById(system, sensorId, &sensor));

    kCheck(GoSensor_Connect(sensor));

    kCheck(kObject_Destroy(system));
    kCheck(kObject_Destroy(api));

    return kOK;
}

int main()
{
    return kSuccess(ExampleMain()) ? 0 : -1;
}
