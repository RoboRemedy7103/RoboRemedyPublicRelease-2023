#include <chrono>
#include <thread>
#include <fmt/format.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>

#include "HPS3DUser_IF.h"

using namespace std;

int g_handle = -1;
static HPS3D_MeasureData_t g_measureData;

long counter = 1;

nt::NetworkTableInstance inst;
std::shared_ptr<nt::NetworkTable> table;
nt::DoublePublisher xPub;
nt::DoubleSubscriber xSub;

nt::DoubleSubscriber regionXSub;
nt::DoubleSubscriber regionYSub;
nt::DoublePublisher zPub;

nt::IntegerPublisher counterPub;

static void EventCallBackFunc(int handle, int eventType, uint8_t *data,
  int dataLen, void *userPara);

int main() {
  inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("Test Client");
  inst.SetServerTeam(7103);

  table = inst.GetTable("lidar");
  xPub = table->GetDoubleTopic("average").Publish();
  xSub = table->GetDoubleTopic("average").Subscribe(-1.0);

  regionXSub = table->GetDoubleTopic("regionX").Subscribe(-2.0);
  regionYSub = table->GetDoubleTopic("regionY").Subscribe(-2.0);
  zPub = table->GetDoubleTopic("z").Publish();

  counterPub = table->GetIntegerTopic("counter").Publish();

  fmt::print("HPS3D160 Demo\n");
  printf("SDK Ver:%s\n", HPS3D_GetSDKVersion());
  while (true) {
    auto ret = HPS3D_MeasureDataInit(&g_measureData);
    if (ret != HPS3D_RET_OK) {
      printf("HPSD_MeasureDataInit failed, Error=%d", ret);
      break;
    }

    ret = HPS3D_USBConnectDevice((char *)"/dev/ttyACM0",&g_handle);
    if (ret == HPS3D_RET_OK) {
      printf("LIDAR Found, Handle = %d\n", g_handle);
    } else {
      printf("No LIDAR Found, Error=%d\n", ret);
      break;
    }

    printf("Version Number: %s\n", HPS3D_GetDeviceVersion(g_handle));
    printf("Serial Number: %s\n", HPS3D_GetSerialNumber(g_handle));

    ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret == HPS3D_RET_OK) {
      printf("Callback Registerd\n");
    } else {
      printf("Callback Failed to Register, Error=%d\n", ret);
      break;
    }

    //HPS3D_EventType_t type = HPS3D_FULL_DEPTH_EVEN;
    //ret = HPS3D_SingleCapture(g_handle, &type, &g_measureData);
    ret = HPS3D_StartCapture(g_handle);
    if (ret == HPS3D_RET_OK) {
      printf("Capture Sucess\n");
      //printf("Avg = %d\n", g_measureData.full_depth_data.distance_average);
    } else {
      printf("Single Capture Failed, Error=%d\n", ret);
    }

    break;
  }

  for (int i=0; i < 100; i++) {
    counterPub.Set(counter);
    counter += 1;
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  fmt::print("Closing.\n");

  if (g_handle != -1) {
    printf("Stopping Capture\n");
    HPS3D_StopCapture(g_handle);
    printf("Closing Device\n");
    HPS3D_CloseDevice(g_handle);
  }

  printf("Freeing Data\n");
  HPS3D_MeasureDataFree(&g_measureData);
}

static void EventCallBackFunc(int handle, int eventTypeInt, uint8_t *data,
  int dataLen, void *userPara)
{
  HPS3D_EventType_t eventType = (HPS3D_EventType_t)eventTypeInt;

  switch (eventType) {
    case HPS3D_FULL_DEPTH_EVEN:
      HPS3D_ConvertToMeasureData(data, &g_measureData, eventType);
      //printf("Got data. Average = %d\n",
             //g_measureData.full_depth_data.distance_average);
      printf("Point 0 Y Value = %f\n",
             g_measureData.full_depth_data.point_cloud_data.point_data[0].y);
      printf("Point 0 X Value = %f\n",
             g_measureData.full_depth_data.point_cloud_data.point_data[0].x);
      printf("Point 0 Z Value = %f\n",
             g_measureData.full_depth_data.point_cloud_data.point_data[0].z);
      xPub.Set(g_measureData.full_depth_data.distance_average);
      inst.Flush();
      break;

    case HPS3D_SYS_EXCEPTION_EVEN:
      printf("Exception Event\n");
      break;

    case HPS3D_DISCONNECT_EVEN:
      printf("Disconnect Event\n");
      break;

    case HPS3D_NULL_EVEN:
    case HPS3D_SIMPLE_ROI_EVEN:
    case HPS3D_FULL_ROI_EVEN:
    case HPS3D_SIMPLE_DEPTH_EVEN:
      printf("Unexpected Event: %d\n", eventTypeInt);
      break;

    default:
      printf("Unknown Event\n");
      break;
  }

}
