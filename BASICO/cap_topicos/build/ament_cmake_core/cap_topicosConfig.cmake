# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cap_topicos_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cap_topicos_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cap_topicos_FOUND FALSE)
  elseif(NOT cap_topicos_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cap_topicos_FOUND FALSE)
  endif()
  return()
endif()
set(_cap_topicos_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cap_topicos_FIND_QUIETLY)
  message(STATUS "Found cap_topicos: 0.0.0 (${cap_topicos_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cap_topicos' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT cap_topicos_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cap_topicos_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cap_topicos_DIR}/${_extra}")
endforeach()
