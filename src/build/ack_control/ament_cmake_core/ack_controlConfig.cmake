# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ack_control_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ack_control_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ack_control_FOUND FALSE)
  elseif(NOT ack_control_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ack_control_FOUND FALSE)
  endif()
  return()
endif()
set(_ack_control_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ack_control_FIND_QUIETLY)
  message(STATUS "Found ack_control: 0.0.0 (${ack_control_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ack_control' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ack_control_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ack_control_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ack_control_DIR}/${_extra}")
endforeach()
