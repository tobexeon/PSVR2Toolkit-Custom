#pragma once

#include <openvr_driver.h>

namespace psvr2_toolkit {

  class HmdDeviceHooks {
  public:
    static void InstallHooks();
    static void UpdateGaze(void* pData, size_t dwSize);
    // [新增] 公开给 IPC Server 调用的接口
    static void EnableCalibration(bool enable);
    static void ReloadCalibration();
  };

} // psvr2_toolkit
