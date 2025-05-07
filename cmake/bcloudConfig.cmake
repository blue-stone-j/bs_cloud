set(bcloud_INCLUDE_DIRS "")
list(APPEND bcloud_INCLUDE_DIRS
  /media/jpw/m10/00hub/bs_cloud/install/include)

file(GLOB_RECURSE bcloud_LIBRARIES
  /media/jpw/m10/00hub/bs_cloud/install/lib/bcloud/*.so)
