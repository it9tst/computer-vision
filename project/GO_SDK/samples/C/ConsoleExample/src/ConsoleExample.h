#include <GoSdk/GoSdk.h>

#define GDK_SAMPLE_CLIENT_DEFAULT_MAIN_ID      (13806)
#define GDK_SAMPLE_CLIENT_DEFAULT_BUDDY_ID     (-1)

typedef struct ConsoleExampleContext
{
    k8u lastMeasurementId;
    GoSystem system;
    k32u mainId;
    k32s buddyId;
    GoSensor mainDevice;
    GoSensor buddyDevice;
    GoDataSet lastDataSet;
} ConsoleExampleContext;

kStatus GdkClientSample_ModeSelection(const ConsoleExampleContext* context);
kStatus GdkClientSample_ConfigureRangeMode(const ConsoleExampleContext* context);
kStatus GdkClientSample_ConfigureProfileMode(const ConsoleExampleContext* context, kBool isResampled);
kStatus GdkClientSample_ConfigureSurfaceMode(const ConsoleExampleContext* context);

kStatus GdkClientSample_ConfigureBuddyDevice(ConsoleExampleContext* context);

kStatus GdkClientSample_AddTools(const ConsoleExampleContext* context);
kStatus GdkClientSample_ConfigureTools(const ConsoleExampleContext* context);
kStatus GdkClientSample_ConfigureExtTool(const ConsoleExampleContext* context, GoExtTool tool);
kStatus GdkClientSample_ConfigureExtParam(const ConsoleExampleContext* context, GoExtParam param);
kStatus GdkClientSample_ConfigureMeasurements(const ConsoleExampleContext* context, GoExtTool tool);
kStatus GdkClientSample_ConfigureMeasurement(const ConsoleExampleContext* context, GoExtMeasurement measurement);

kStatus GdkClientSample_ConfigureExtToolParams(const ConsoleExampleContext* context, GoExtTool tool);
kStatus GdkClientSample_ConfigureProfileRegion(const ConsoleExampleContext* context, GoExtParamProfileRegion param);
kStatus GdkClientSample_ConfigureSurfaceRegion2d(const ConsoleExampleContext* context, GoExtParamSurfaceRegion2d param);
kStatus GdkClientSample_ConfigureSurfaceRegion3d(const ConsoleExampleContext* context, GoExtParamSurfaceRegion3d param);

kStatus GdkClientSample_StartStopSensor(ConsoleExampleContext* context);
kStatus kCall GdkClientSample_DataHandler(kPointer context, GoSensor sensor, GoDataSet dataSet);
kStatus GdkClientSample_DisplayLastMeasurementResults(ConsoleExampleContext* context);

