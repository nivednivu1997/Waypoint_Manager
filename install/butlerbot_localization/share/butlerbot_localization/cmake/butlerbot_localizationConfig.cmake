# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_butlerbot_localization_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED butlerbot_localization_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(butlerbot_localization_FOUND FALSE)
  elseif(NOT butlerbot_localization_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(butlerbot_localization_FOUND FALSE)
  endif()
  return()
endif()
set(_butlerbot_localization_CONFIG_INCLUDED TRUE)

# output package information
if(NOT butlerbot_localization_FIND_QUIETLY)
  message(STATUS "Found butlerbot_localization: 0.0.0 (${butlerbot_localization_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'butlerbot_localization' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${butlerbot_localization_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(butlerbot_localization_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${butlerbot_localization_DIR}/${_extra}")
endforeach()
