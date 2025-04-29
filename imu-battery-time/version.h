// version.h - 自動產生的版本資訊
// 生成時間: 2025-04-29 22:49:56
// 請勿手動修改此檔案，它將在建置時自動更新
#ifndef VERSION_H
#define VERSION_H

// Version information
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_STRING "1.0.0"
#define VERSION_STRING_FULL "1.0.0-0648837"

// Build information
#define BUILD_TIMESTAMP "2025-04-29 22:49:56"
#define GIT_TAG "v1.0"
#define GIT_COMMIT "0648837"
#define GIT_AVAILABLE true

// 將版本號添加到BLE廣播數據中的便捷函數
String getVersionForAdvertising() {
  return String(VERSION_MAJOR) + "." + 
         String(VERSION_MINOR) + "." +
         String(VERSION_PATCH);
}

// 增加一個版本控制結構體，方便後續功能擴展
typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  const char* gitTag;
  const char* gitCommit;
  const char* buildTimestamp;
  bool gitAvailable;
} version_info_t;

// 獲取完整版本資訊
version_info_t getVersionInfo() {
  version_info_t info = {
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_PATCH,
    GIT_TAG,
    GIT_COMMIT,
    BUILD_TIMESTAMP,
    GIT_AVAILABLE
  };
  return info;
}

#endif // VERSION_H
