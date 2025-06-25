#ifndef GLOBAL_DEFINITION_H
#define GLOBAL_DEFINITION_H

#include <filesystem>
#include <string>

const std::string SYSTEM_VERSION   = "Linux";
const std::string SOFTWARE_VERSION = "Linux";
const std::string RELEASE_DATE     = "22.06.25";

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <vector>
static std::string getExePath()
{
  char result[MAX_PATH];
  GeModuleFileName(NULL, result, MAX_PATH);
  return std::string(result);
}
#elif defined(__linux__)
#include <unistd.h>
static std::string getExePath()
{
  constexpr int len = 256;
  char arr_tmp[len] = {0};

  int n = readlink("/proc/self/exe", arr_tmp, len);
  if (n >= len || n == -1)
  {
    // LOG(WARNING) << "Failed to read the executable path";
    return "";
  }

  std::string exePath(arr_tmp, n);
  std::string::size_type pos = exePath.find_last_of('/');
  if (pos == std::string::npos)
  {
    return "";
  }
  return exePath;
}
#endif

static std::string getExeDir()
{
  return std::filesystem::path(getExePath()).parent_path().string();
}

static std::string getRootDir()
{
  return std::filesystem::path(getExeDir()).parent_path().string();
}
const std::string WORK_SPACE_PATH = getRootDir();

#endif
