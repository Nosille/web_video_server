#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "web_video_server::web_video_server" for configuration ""
set_property(TARGET web_video_server::web_video_server APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(web_video_server::web_video_server PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libweb_video_server.so"
  IMPORTED_SONAME_NOCONFIG "libweb_video_server.so"
  )

list(APPEND _cmake_import_check_targets web_video_server::web_video_server )
list(APPEND _cmake_import_check_files_for_web_video_server::web_video_server "${_IMPORT_PREFIX}/lib/libweb_video_server.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
