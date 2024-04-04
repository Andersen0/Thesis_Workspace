# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_copilot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED copilot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(copilot_FOUND FALSE)
  elseif(NOT copilot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(copilot_FOUND FALSE)
  endif()
  return()
endif()
set(_copilot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT copilot_FIND_QUIETLY)
  message(STATUS "Found copilot: 0.0.0 (${copilot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'copilot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${copilot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(copilot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${copilot_DIR}/${_extra}")
endforeach()
