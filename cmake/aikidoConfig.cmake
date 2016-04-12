set(AIKIDO_VERSION x.y.z)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was aikidoConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include("${CMAKE_CURRENT_LIST_DIR}/aikidoTargets.cmake")

set_and_check(aikido_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set(aikido_LIBRARY aikido)

# Transitive dependencies.
find_package(Boost REQUIRED)
find_package(DART REQUIRED COMPONENTS core)
find_package(OMPL REQUIRED)

# Both direct and transitive dependencies.
set(aikido_INCLUDE_DIRS
  ${aikido_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
set(aikido_LIBRARIES
  ${aikido_LIBRARY}
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
)
