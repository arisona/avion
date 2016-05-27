cmake_minimum_required(VERSION 3.5)

project(avion)


######## Set output paths
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})


######## Set language options
set(CMAKE_CXX_STANDARD 11)


######### Add java
find_package(Java REQUIRED)
find_package(JNI REQUIRED)
include(UseJava)
include_directories("${JNI_INCLUDE_DIRS}")


######### Add library dependencies
if (APPLE)
  FIND_LIBRARY(FOUNDATION_LIBRARY Foundation)
  FIND_LIBRARY(AVFOUNDATION_LIBRARY AVFoundation)
  FIND_LIBRARY(COREMEDIA_LIBRARY CoreMedia)
  FIND_LIBRARY(COREVIDEO_LIBRARY CoreVideo)
  FIND_LIBRARY(JAVAVM_LIBRARY JavaVM)
  SET(PLATFORM_LIBS
    ${FOUNDATION_LIBRARY}
    ${AVFOUNDATION_LIBRARY}
    ${COREMEDIA_LIBRARY}
    ${COREVIDEO_LIBRARY}
    ${JAVAVM_LIBRARY}
  )
elseif (WINDOWS)

elseif (UNIX)

endif()
message (STATUS "PLATFORM_LIBS=${PLATFORM_LIBS}")


######## Add JNI / Javah target
create_javah(TARGET javah
  CLASSES ch.fhnw.ether.avion.Avion
  CLASSPATH ${CMAKE_SOURCE_DIR}/../src/main/java
  OUTPUT_DIR ${CMAKE_SOURCE_DIR}/generated
)
include_directories(${PROJECT_SOURCE_DIR}/generated)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/generated)


######## Add avion include location
include_directories(${PROJECT_SOURCE_DIR}/avion)


######## Add avion library sources
if (APPLE)
  SET(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.mm
    ${PROJECT_SOURCE_DIR}/avion/avion_avfoundation.hpp
  )
elseif (WINDOWS)
  SET(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.cpp
    ${PROJECT_SOURCE_DIR}/avion/avion_mediafoundation.hpp
  )
elseif (UNIX)
  SET(PLATFORM_SOURCES
    ${PROJECT_SOURCE_DIR}/avion/avion.cpp
    ${PROJECT_SOURCE_DIR}/avion/avion_ffmpeg.hpp
  )
endif()

add_library(avion
  SHARED
  ${PROJECT_SOURCE_DIR}/avion/avion.hpp
  ${PROJECT_SOURCE_DIR}/avion/audio_buffer.hpp
  ${PROJECT_SOURCE_DIR}/avion/avion_jni.cpp
  ${PLATFORM_SOURCES}
)
add_dependencies(avion javah)
target_link_libraries(avion ${PLATFORM_LIBS})

if (APPLE)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-macos)
elseif (WINDOWS)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-windows)
elseif (UNIX)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-linux)
endif()


######### Add avion test executable
add_executable(avion_test
  ${PROJECT_SOURCE_DIR}/avion_test/avion_test.cpp
)
target_link_libraries(avion_test avion)


######### Install avion library so it can be packaged by gradle build
install(TARGETS avion
  LIBRARY
  DESTINATION ${PROJECT_SOURCE_DIR}/../libs
)

######## Finally, set build type (for use with make)
set(CMAKE_BUILD_TYPE Release)
