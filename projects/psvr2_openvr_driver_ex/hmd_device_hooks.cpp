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
#include <fstream>
#include <vector>
#include <cmath>
#include <shlobj.h> // 用于获取文档路径

namespace psvr2_toolkit {

  // 全局变量
  float g_calibOffsetX = 0.0f;
  float g_calibOffsetY = 0.0f;
  bool  g_hasCalibrated = false;
  bool  g_isCalibratingNow = false; // [新增] 是否正在校准中

  // [修改] 获取校准文件路径 (用户文档目录)
  std::string GetCalibrationFilePath() {
      char path[MAX_PATH];
      if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_MYDOCUMENTS, NULL, 0, path))) {
          return std::string(path) + "\\PSVR2Calibration.txt";
      }
      return "C:\\PSVR2Calibration.txt"; // 回退路径
  }

  void LoadCalibrationData() {
      std::string path = GetCalibrationFilePath();
      std::ifstream file(path);
      if (file.is_open()) {
          if (file >> g_calibOffsetX >> g_calibOffsetY) {
              g_hasCalibrated = true;
              Util::DriverLog("[Gaze Calibration] Loaded Offset X: %f, Y: %f from %s", g_calibOffsetX, g_calibOffsetY, path.c_str());
          } else {
              Util::DriverLog("[Gaze Calibration] Invalid file format.");
          }
          file.close();
      } else {
          g_hasCalibrated = false;
          Util::DriverLog("[Gaze Calibration] No calibration file found at %s", path.c_str());
      }
  }

  // [新增] 实现接口
  void HmdDeviceHooks::EnableCalibration(bool enable) {
      g_isCalibratingNow = !enable; // 如果 enable=true(应用校准)，则 isCalibrating=false
      Util::DriverLog("[Gaze Calibration] Calibration Active: %d", enable);
  }

  void HmdDeviceHooks::ReloadCalibration() {
      LoadCalibrationData();
      g_isCalibratingNow = false; // 读取完成后，恢复应用校准状态
  }

#ifdef OPENVR_EXTENSIONS_AVAILABLE
  void* g_pOpenVRExHandle = nullptr;
#endif
  vr::VRInputComponentHandle_t eyeTrackingComponent;

  // ... (中间的 Activate/Deactivate Hook 代码保持不变，省略以节省篇幅，请保留原有的 Activate/Deactivate 代码) ...
  // 为确保代码完整性，这里我只列出 UpdateGaze 和 InstallHooks 的修改，其他 Hook 函数请保持原样。
  
  vr::EVRInitError(*sie__psvr2__HmdDevice__Activate)(void*, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void* thisptr, uint32_t unObjectId) {
      // ... (保留你之前的 ActivateHook 代码) ...
      vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
      vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
      if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE)) {
        vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
      }
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplayAllowNightMode_Bool, true);
      vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DashboardScale_Float, .9f);
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);
      if (vr::VRDriverInput()) {
        (vr::VRDriverInput())->CreateEyeTrackingComponent(ulPropertyContainer, "/eyetracking", &eyeTrackingComponent);
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
      if (g_pOpenVRExHandle) psvr2_toolkit::openvr_ex::OnHmdDeactivate(&g_pOpenVRExHandle);
  #endif
  }

  void* (*CaesarManager__getInstance)();
  uint64_t(*CaesarManager__getIMUTimestampOffset)(void* thisptr, int64_t* hmdToHostOffset);

  inline const int64_t GetHostTimestamp() {
      static LARGE_INTEGER frequency{};
      if (frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
      LARGE_INTEGER now;
      QueryPerformanceCounter(&now);
      return static_cast<int64_t>((static_cast<double>(now.QuadPart) / static_cast<double>(frequency.QuadPart)) * 1e6);
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

    // --- [修改] 校准核心逻辑 ---
    float rawX = direction.x;
    float rawY = direction.y;
    float rawZ = direction.z;

    // 只有在“已加载校准文件”且“当前不在运行校准程序”时，才应用偏移
    if (g_hasCalibrated && !g_isCalibratingNow) {
        rawX += g_calibOffsetX;
        rawY += g_calibOffsetY;
        
        float length = std::sqrt(rawX * rawX + rawY * rawY + rawZ * rawZ);
        if (length > 0.0001f) {
            rawX /= length;
            rawY /= length;
            rawZ /= length;
        }
    }
    // -------------------------

    eyeTrackingData.vGazeOrigin = vr::HmdVector3_t{ -origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f };
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
    LoadCalibrationData(); // 启动时读取

    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x19D1B0),
                         reinterpret_cast<void*>(sie__psvr2__HmdDevice__ActivateHook),
                         reinterpret_cast<void**>(&sie__psvr2__HmdDevice__Activate));
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x19EBF0),
                         reinterpret_cast<void*>(sie__psvr2__HmdDevice__DeactivateHook),
                         reinterpret_cast<void**>(&sie__psvr2__HmdDevice__Deactivate));

    CaesarManager__getInstance = decltype(CaesarManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x124c90);
    CaesarManager__getIMUTimestampOffset = decltype(CaesarManager__getIMUTimestampOffset)(pHmdDriverLoader->GetBaseAddress() + 0x1252e0);
  }

} // psvr2_toolkit
