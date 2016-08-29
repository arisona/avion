cmake_minimum_required(VERSION 3.5)

project(avion)


######## Set output paths
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})


######## Set language options
set(CMAKE_CXX_STANDARD 11)


######## Set "LINUX" (to avoid ambiguity between APPLE and UNIX)
if("${CMAKE_SYSTEM}" MATCHES "Linux")
  set(LINUX true)
endif()


######### Add java
find_package(Java REQUIRED)
find_package(JNI REQUIRED)
include(UseJava)
include_directories("${JNI_INCLUDE_DIRS}")


######### Add library dependencies
if (APPLE)
  find_library(FOUNDATION_LIBRARY Foundation )
  find_library(AVFOUNDATION_LIBRARY AVFoundation)
  find_library(COREMEDIA_LIBRARY CoreMedia)
  find_library(COREVIDEO_LIBRARY CoreVideo)
  find_library(JAVAVM_LIBRARY JavaVM)
  set(PLATFORM_LIBS
    ${FOUNDATION_LIBRARY}
    ${AVFOUNDATION_LIBRARY}
    ${COREMEDIA_LIBRARY}
    ${COREVIDEO_LIBRARY}
    ${JAVAVM_LIBRARY}
  )
  mark_as_advanced(FORCE
    FOUNDATION_LIBRARY
    AVFOUNDATION_LIBRARY
    COREMEDIA_LIBRARY
    COREVIDEO_LIBRARY
    JAVAVM_LIBRARY
  )
elseif (WIN32)
  set(PLATFORM_LIBS "")
elseif (LINUX)
  set(PLATFORM_LIBS "")
endif()

message (STATUS "PLATFORM_LIBS=${PLATFORM_LIBS}")


######## Add JNI / Javah target
add_custom_target(make_generated_directory ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/generated)

create_javah(TARGET javah
  CLASSES ch.fhnw.ether.avion.Avion
  CLASSPATH ${CMAKE_SOURCE_DIR}/../src/main/java
  OUTPUT_DIR ${CMAKE_SOURCE_DIR}/generated
  DEPENDS make_generated_directory
)
include_directories(${PROJECT_SOURCE_DIR}/generated)
# file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/generated)
message(STATUS ${PROJECT_SOURCE_DIR}/generated)


######## Add ffmpeg
option(FFMPEG_ENABLED "FFMPEG enabled" OFF)
if (FFMPEG_ENABLED OR LINUX)
  #set(FFMPEG_ROOT "/usr" CACHE FILEPATH "FFMPEG root directory")
  #set(FFMPEG_ROOT "/Users/radar/devel/lib/ffmpeg" CACHE FILEPATH "FFMPEG root directory")
  set(FFMPEG_ROOT "E:/win64/ffmpeg" CACHE FILEPATH "FFMPEG root directory")
  include_directories(${FFMPEG_ROOT}/include)
  find_library(AVCODEC_LIBRARY avcodec PATHS ${FFMPEG_ROOT}/lib)
  find_library(AVDEVICE_LIBRARY avdevice PATHS ${FFMPEG_ROOT}/lib)
  find_library(AVFORMAT_LIBRARY avformat PATHS ${FFMPEG_ROOT}/lib)
  find_library(AVUTIL_LIBRARY avutil PATHS ${FFMPEG_ROOT}/lib)
  find_library(SWRESAMPLE_LIBRARY swresample PATHS ${FFMPEG_ROOT}/lib)
  find_library(SWSCALE_LIBRARY swscale PATHS ${FFMPEG_ROOT}/lib)
  set(FFMPEG_LIBS
    ${AVCODEC_LIBRARY}
    ${AVDEVICE_LIBRARY}
    ${AVFORMAT_LIBRARY}
    ${AVUTIL_LIBRARY}
    ${SWRESAMPLE_LIBRARY}
    ${SWSCALE_LIBRARY}
  )
  mark_as_advanced(FORCE
    AVCODEC_LIBRARY
    AVDEVICE_LIBRARY
    AVFORMAT_LIBRARY
    AVUTIL_LIBRARY
    SWRESAMPLE_LIBRARY
    SWSCALE_LIBRARY
  )
  set(FFMPEG_SOURCES ${PROJECT_SOURCE_DIR}/avion/avion_ffmpeg.hpp)
  set(FFMPEG_LIBS ${FFMPEG_LIBS})
  add_definitions("-DFFMPEG_ENABLED")
else()
  set(FFMPEG_SOURCES "")
  set(FFMPEG_LIBS "")
endif()

message (STATUS "FFMPEG_LIBS=${FFMPEG_LIBS}")


######## Add avion include location
include_directories(${PROJECT_SOURCE_DIR}/avion)


######## Add avion library sources
if (APPLE)
  set(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.mm
    ${PROJECT_SOURCE_DIR}/avion/avion_avfoundation.hpp
  )
elseif (WIN32)
  set(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.cpp
    ${PROJECT_SOURCE_DIR}/avion/avion_mediafoundation.hpp
  )
elseif (LINUX)
  set(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.cpp
  )
endif()

add_library(avion
  SHARED
  ${PROJECT_SOURCE_DIR}/avion/avion.hpp
  ${PROJECT_SOURCE_DIR}/avion/util.hpp
  ${PROJECT_SOURCE_DIR}/avion/avion_jni.cpp
  ${PROJECT_SOURCE_DIR}/generated/ch_fhnw_ether_avion_Avion.h
  ${PLATFORM_SOURCES}
  ${FFMPEG_SOURCES}
)
add_dependencies(avion javah)
target_link_libraries(avion ${PLATFORM_LIBS} ${FFMPEG_LIBS})

if (APPLE)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-macos)
elseif (WIN32)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-windows-64bit)
elseif (LINUX)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-linux-64bit)
endif()


######### Add avion test executable
add_executable(avion_test
  ${PROJECT_SOURCE_DIR}/avion_test/avion_test.cpp
)
target_link_libraries(avion_test avion)


######### Install avion library so it can be packaged by gradle build
install(TARGETS avion
#  LIBRARY
  DESTINATION "${PROJECT_SOURCE_DIR}/../libs"
)

######## Finally, set build type (for use with make)
set(CMAKE_BUILD_TYPE Release)
