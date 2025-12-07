#ifdef OPENVR_EXTENSIONS_AVAILABLE
#include "psvr2_openvr_driver/openvr_ex/openvr_ex.h"
#endif

#include "hmd2_gaze.h"

#include "hmd_device_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <cstdint>
// [新增] 头文件
#include <fstream>
#include <vector>
#include <cmath>

namespace psvr2_toolkit {

  // [新增] 全局校准变量
  float g_calibOffsetX = 0.0f;
  float g_calibOffsetY = 0.0f;
  bool  g_hasCalibrated = false;

  // [新增] 读取校准文件函数
  void LoadCalibrationData() {
      std::string path = "C:\\PSVR2Calibration.txt";
      std::ifstream file(path);
      if (file.is_open()) {
          if (file >> g_calibOffsetX >> g_calibOffsetY) {
              g_hasCalibrated = true;
              Util::DriverLog("[Gaze Calibration] Loaded Offset X: %f, Y: %f", g_calibOffsetX, g_calibOffsetY);
          } else {
              Util::DriverLog("[Gaze Calibration] Invalid file format.");
          }
          file.close();
      } else {
          Util::DriverLog("[Gaze Calibration] No calibration file found at %s. Using raw data.", path.c_str());
      }
  }

#ifdef OPENVR_EXTENSIONS_AVAILABLE
  void* g_pOpenVRExHandle = nullptr;
#endif
  vr::VRInputComponentHandle_t eyeTrackingComponent;

  vr::EVRInitError(*sie__psvr2__HmdDevice__Activate)(void*, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void* thisptr, uint32_t unObjectId) {
    vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
    vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    // Tell SteamVR we want the chaperone visibility disabled if we're actually disabling the chaperone.
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE)) {
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
    }

    // Tell SteamVR to allow runtime framerate changes.
    // SteamVR does not allow this feature on AMD GPUs, so this is NVIDIA-only currently.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);

    // Tell SteamVR to allow night mode setting.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplayAllowNightMode_Bool, true);

    // Tell SteamVR our dashboard scale.
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DashboardScale_Float, .9f);

    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);

    if (vr::VRDriverInput())
    {
      vr::EVRInputError result = (vr::VRDriverInput())->CreateEyeTrackingComponent(ulPropertyContainer, "/eyetracking", &eyeTrackingComponent);
      if (result != vr::VRInputError_None) {
        vr::VRDriverLog()->Log("Failed to create eye tracking component.");
      }
    }
    else
    {
      vr::VRDriverLog()->Log("Failed to get driver input interface. Are you on the latest version of SteamVR?");
    }

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    psvr2_toolkit::openvr_ex::OnHmdActivate(ulPropertyContainer, &g_pOpenVRExHandle);
#endif

    return result;
  }

  void (*sie__psvr2__HmdDevice__Deactivate)(void*) = nullptr;
  void sie__psvr2__HmdDevice__DeactivateHook(void* thisptr) {
    sie__psvr2__HmdDevice__Deactivate(thisptr);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    if (g_pOpenVRExHandle) {
      psvr2_toolkit::openvr_ex::OnHmdDeactivate(&g_pOpenVRExHandle);
    }
#endif
  }

  void* (*CaesarManager__getInstance)();
  uint64_t(*CaesarManager__getIMUTimestampOffset)(void* thisptr, int64_t* hmdToHostOffset);

  inline const int64_t GetHostTimestamp()
  {
    static LARGE_INTEGER frequency{};
    if (frequency.QuadPart == 0)
    {
      QueryPerformanceFrequency(&frequency);
    }

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    return static_cast<int64_t>((static_cast<double>(now.QuadPart) /
      static_cast<double>(frequency.QuadPart)) * 1e6);
  }

  void HmdDeviceHooks::UpdateGaze(void* pData, size_t dwSize)
  {
    Hmd2GazeState* pGazeState = reinterpret_cast<Hmd2GazeState*>(pData);
    vr::VREyeTrackingData_t eyeTrackingData{};

    bool valid = pGazeState->combined.isGazeDirValid;

    eyeTrackingData.bActive = valid;
    eyeTrackingData.bTracked = valid;
    eyeTrackingData.bValid = valid;

    auto& origin = pGazeState->combined.gazeOriginMm;
    auto& direction = pGazeState->combined.gazeDirNorm;

    // --- [修改] 校准逻辑开始 ---
    float rawX = direction.x;
    float rawY = direction.y;
    float rawZ = direction.z;

    if (g_hasCalibrated) {
        // 应用校准文件中的偏移量
        // 注意：这里的正负号可能需要根据实际体验调整
        // 假设校准程序输出的是“注视点需要移动多少才能对准”，那么这里应该是 +
        rawX += g_calibOffsetX;
        rawY += g_calibOffsetY;
        
        // 重新归一化向量，保证数据有效性
        float length = std::sqrt(rawX * rawX + rawY * rawY + rawZ * rawZ);
        if (length > 0.0001f) {
            rawX /= length;
            rawY /= length;
            rawZ /= length;
        }
    }
    // --- [修改] 校准逻辑结束 ---

    eyeTrackingData.vGazeOrigin = vr::HmdVector3_t{ -origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f };
    // 使用校准后的 rawX, rawY, rawZ
    eyeTrackingData.vGazeTarget = vr::HmdVector3_t{ -rawX, rawY, -rawZ };

    int64_t hmdToHostOffset;

    CaesarManager__getIMUTimestampOffset(CaesarManager__getInstance(), &hmdToHostOffset);

    double timeOffset = ((static_cast<int64_t>(pGazeState->combined.timestamp) + hmdToHostOffset) - GetHostTimestamp()) / 1e6;

    (vr::VRDriverInput())->UpdateEyeTrackingComponent(eyeTrackingComponent, &eyeTrackingData, timeOffset);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    if (g_pOpenVRExHandle) {
      psvr2_toolkit::openvr_ex::OnHmdUpdate(&g_pOpenVRExHandle, pData, dwSize);
    }
#endif
  }

  void HmdDeviceHooks::InstallHooks() {
    // [新增] 在安装钩子（即驱动初始化阶段）读取校准数据
    LoadCalibrationData();

    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();

    // sie::psvr2::HmdDevice::Activate
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x19D1B0),
                         reinterpret_cast<void*>(sie__psvr2__HmdDevice__ActivateHook),
                         reinterpret_cast<void**>(&sie__psvr2__HmdDevice__Activate));

    // sie::psvr2::HmdDevice::Deactivate
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x19EBF0),
                         reinterpret_cast<void*>(sie__psvr2__HmdDevice__DeactivateHook),
                         reinterpret_cast<void**>(&sie__psvr2__HmdDevice__Deactivate));

    CaesarManager__getInstance = decltype(CaesarManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x124c90);
    CaesarManager__getIMUTimestampOffset = decltype(CaesarManager__getIMUTimestampOffset)(pHmdDriverLoader->GetBaseAddress() + 0x1252e0);
  }

} // psvr2_toolkit
