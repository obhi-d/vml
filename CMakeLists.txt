cmake_minimum_required(VERSION 3.1)

project(vml VERSION 0.0.1 LANGUAGES CXX)


option(VML_BUILD_TESTS "Build the unit tests when BUILD_TESTING is enabled." ON)
option(VML_USE_SSE_AVX "Use SSE and AVX instructions" ON)
option(VML_USE_SSE_LEVEL_4 "SSE instruction level" OFF)
option(VML_USE_SSE_LEVEL_2 "SSE instruction level 2" OFF)
option(VML_PREFER_SPEED_OVER_ACCURACY "Prefer speed over accuracy" ON)
option(VML_BUILD_DOCS "Build documentation" ON)
##
## CONFIGURATION
##
set(VML_TARGET_NAME               ${PROJECT_NAME})
set(VML_INCLUDE_BUILD_DIR "${PROJECT_SOURCE_DIR}/include")

if(VML_USE_SSE_AVX)
  set(VML_USE_SSE_AVX 1)
else(VML_USE_SSE_AVX)
  set(VML_USE_SSE_AVX 0)
endif(VML_USE_SSE_AVX)

if(VML_USE_SSE_LEVEL_4)
  set(VML_USE_SSE_LEVEL 4)
else(VML_USE_SSE_LEVEL_4)
if(VML_USE_SSE_LEVEL_2)
  set(VML_USE_SSE_LEVEL 2)
else()
  set(VML_USE_SSE_LEVEL 3)
endif()
endif(VML_USE_SSE_LEVEL_4)

if(VML_PREFER_SPEED_OVER_ACCURACY)
  set(VML_PREFER_SPEED_OVER_ACCURACY 1)
else(VML_PREFER_SPEED_OVER_ACCURACY)
  set(VML_PREFER_SPEED_OVER_ACCURACY 0)
endif(VML_PREFER_SPEED_OVER_ACCURACY)
##
## TARGET
##
message("Target name: ${VML_TARGET_NAME}")
add_library(${VML_TARGET_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${VML_TARGET_NAME} ALIAS ${VML_TARGET_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

target_compile_definitions(
    ${VML_TARGET_NAME}
    INTERFACE -DVML_USE_SSE_AVX=${VML_USE_SSE_AVX} -DVML_USE_SSE_LEVEL=${VML_USE_SSE_LEVEL} -DVML_PREFER_SPEED_OVER_ACCURACY=${VML_PREFER_SPEED_OVER_ACCURACY}
)

target_include_directories(
    ${VML_TARGET_NAME}
    INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

##
## TESTS
##
include(CTest) #adds option BUILD_TESTING (default ON)

if(BUILD_TESTING AND VML_BUILD_TESTS)
    enable_testing()
    add_subdirectory(unit_tests)
endif()

##
## INSTALL
##
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY SameMajorVersion)

if(DEFINED CMAKE_INSTALL_DATAROOTDIR)
    configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)


    install(EXPORT ${PROJECT_NAME}_Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

    install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
            DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)
endif()

find_package(Doxygen)
if (VML_BUILD_DOCS)
if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
    # build the docs

  if (DOXYGEN_FOUND)
      # set input and output files
      set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxygen.in)
      set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

      # request to configure the file
      configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
      message("Doxygen build started")

      # note the option ALL which allows to build the docs together with the application
      add_custom_target( Documentation ALL
          COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
          COMMENT "Generating API documentation with Doxygen"
          VERBATIM )
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND)

endif()
endif(VML_BUILD_DOCS)

install(DIRECTORY include DESTINATION .)
