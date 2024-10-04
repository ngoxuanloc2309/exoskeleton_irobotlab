# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tongquan_sldasm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tongquan_sldasm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tongquan_sldasm_FOUND FALSE)
  elseif(NOT tongquan_sldasm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tongquan_sldasm_FOUND FALSE)
  endif()
  return()
endif()
set(_tongquan_sldasm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tongquan_sldasm_FIND_QUIETLY)
  message(STATUS "Found tongquan_sldasm: 1.0.0 (${tongquan_sldasm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tongquan_sldasm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tongquan_sldasm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tongquan_sldasm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tongquan_sldasm_DIR}/${_extra}")
endforeach()
