// version.h - �۰ʲ��ͪ�������T
// �ͦ��ɶ�: 2025-06-05 22:25:00
// �ФŤ�ʭק惡�ɮסA���N�b�ظm�ɦ۰ʧ�s
#ifndef VERSION_H
#define VERSION_H

// Version information
#define VERSION_MAJOR 2
#define VERSION_MINOR 0
#define VERSION_PATCH 8
#define VERSION_STRING "2.0.8"
#define VERSION_STRING_FULL "2.0.8-a1c87a5"

// Build information
#define BUILD_TIMESTAMP "2025-06-05 22:25:00"
#define GIT_TAG "2.0"
#define GIT_COMMIT "a1c87a5"
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
