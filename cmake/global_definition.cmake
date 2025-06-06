set(SOFTWARE_VERSION ${BMATH_VERSION})

string(TIMESTAMP RELEASE_DATE "%d.%m.%y")

if(_WIN_)
  set(SYSTEM_VERSION Windows)
elseif(UNIX AND NOT APPLE)
  set(SYSTEM_VERSION Linux)
elseif(APPLE)
  set(SYSTEM_VERSION macOS)
else()
  set(SYSTEM_VERSION Unknow)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/common/global_definition.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/common/global_definition.h
)

