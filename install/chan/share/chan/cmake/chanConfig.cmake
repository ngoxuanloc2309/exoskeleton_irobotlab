# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_chan_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED chan_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(chan_FOUND FALSE)
  elseif(NOT chan_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(chan_FOUND FALSE)
  endif()
  return()
endif()
set(_chan_CONFIG_INCLUDED TRUE)

# output package information
if(NOT chan_FIND_QUIETLY)
  message(STATUS "Found chan: 0.3.0 (${chan_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'chan' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${chan_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(chan_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${chan_DIR}/${_extra}")
endforeach()
