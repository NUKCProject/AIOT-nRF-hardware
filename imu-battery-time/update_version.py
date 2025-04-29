#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# update_version.py - 自動更新版本資訊

import os
import subprocess
import datetime
import sys

def command_exists(command):
    """檢查命令是否存在"""
    try:
        subprocess.check_call([command, "--version"], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
        return True
    except (FileNotFoundError, subprocess.SubprocessError):
        return False

def get_git_tag():
    """獲取最近的Git標籤"""
    try:
        return subprocess.check_output(
            ["git", "describe", "--tags", "--abbrev=0"], 
            stderr=subprocess.DEVNULL
        ).strip().decode("utf-8")
    except subprocess.SubprocessError:
        return "v1.0"

def get_commit_count(git_tag):
    """獲取從指定標籤到HEAD的提交數量"""
    try:
        count = subprocess.check_output(
            ["git", "rev-list", "--count", f"{git_tag}..HEAD"],
            stderr=subprocess.DEVNULL
        ).strip().decode("utf-8")
        return int(count) if count else 0
    except subprocess.SubprocessError:
        return 0

def get_commit_hash():
    """獲取當前提交的短哈希值"""
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL
        ).strip().decode("utf-8")
    except subprocess.SubprocessError:
        return "unknown"

def main():
    # 檢查Git是否可用
    git_available = command_exists("git")
    
    if git_available:
        # Git可用，從Git獲取版本資訊
        git_tag = get_git_tag()
        
        # 移除v字首（如果有）
        version = git_tag[1:] if git_tag.startswith('v') else git_tag
        
        # 解析主版本號和次版本號
        parts = version.split('.')
        if len(parts) >= 2:
            major = parts[0]
            minor = parts[1]
        else:
            major = "1"
            minor = "0"
        
        # 從最近的標籤到HEAD的提交數量作為修訂號
        patch = str(get_commit_count(git_tag))
        
        # 獲取提交哈希
        commit_hash = get_commit_hash()
        
        print(f"使用Git版本資訊: v{major}.{minor}.{patch} ({git_tag} + {patch} commits)")
    else:
        # Git不可用，使用預設版本
        major = "1"
        minor = "0"
        patch = "0"
        git_tag = "v1.0"
        commit_hash = "nogit"
        
        print(f"警告: Git不可用，使用預設版本號 v{major}.{minor}.{patch}")
    
    # 獲取當前日期時間
    build_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # 構造完整版本字串
    version_string = f"{major}.{minor}.{patch}"
    version_string_full = f"{version_string}-{commit_hash}"
    
    # 版本標頭文件內容
    version_h = f"""// version.h - 自動產生的版本資訊
// 生成時間: {build_date}
// 請勿手動修改此檔案，它將在建置時自動更新
#ifndef VERSION_H
#define VERSION_H

// Version information
#define VERSION_MAJOR {major}
#define VERSION_MINOR {minor}
#define VERSION_PATCH {patch}
#define VERSION_STRING "{version_string}"
#define VERSION_STRING_FULL "{version_string_full}"

// Build information
#define BUILD_TIMESTAMP "{build_date}"
#define GIT_TAG "{git_tag}"
#define GIT_COMMIT "{commit_hash}"
#define GIT_AVAILABLE {str(git_available).lower()}

// 將版本號添加到BLE廣播數據中的便捷函數
String getVersionForAdvertising() {{
  return String(VERSION_MAJOR) + "." + 
         String(VERSION_MINOR) + "." +
         String(VERSION_PATCH);
}}

// 增加一個版本控制結構體，方便後續功能擴展
typedef struct {{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  const char* gitTag;
  const char* gitCommit;
  const char* buildTimestamp;
  bool gitAvailable;
}} version_info_t;

// 獲取完整版本資訊
version_info_t getVersionInfo() {{
  version_info_t info = {{
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_PATCH,
    GIT_TAG,
    GIT_COMMIT,
    BUILD_TIMESTAMP,
    GIT_AVAILABLE
  }};
  return info;
}}

#endif // VERSION_H
"""
    
    # 寫入版本標頭文件
    with open("version.h", "w") as f:
        f.write(version_h)
    
    print(f"版本檔案已更新: v{version_string} ({commit_hash})")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())