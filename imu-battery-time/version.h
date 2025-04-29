// version.h - �۰ʲ��ͪ�������T
// �ͦ��ɶ�: 2025-04-29 22:49:56
// �ФŤ�ʭק惡�ɮסA���N�b�ظm�ɦ۰ʧ�s
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

// �N�������K�[��BLE�s���ƾڤ����K�����
String getVersionForAdvertising() {
  return String(VERSION_MAJOR) + "." + 
         String(VERSION_MINOR) + "." +
         String(VERSION_PATCH);
}

// �W�[�@�Ӫ�������c��A��K����\���X�i
typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  const char* gitTag;
  const char* gitCommit;
  const char* buildTimestamp;
  bool gitAvailable;
} version_info_t;

// ������㪩����T
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
