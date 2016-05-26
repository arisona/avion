cmake_minimum_required(VERSION 3.5)

project(avion)

# Set output paths
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Set language options
set(CMAKE_CXX_STANDARD 11)

# Add packages

find_package(Java REQUIRED)
find_package(JNI REQUIRED)
include(UseJava)
#message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
#message (STATUS "JNI_LIBRARIES=${JNI_LIBRARIES}")

# Add search paths
include_directories("${JNI_INCLUDE_DIRS}")

# Add library dependencies
if (APPLE)
  FIND_LIBRARY(FOUNDATION_LIBRARY Foundation)
  FIND_LIBRARY(AVFOUNDATION_LIBRARY AVFoundation)
  FIND_LIBRARY(COREMEDIA_LIBRARY CoreMedia)
  FIND_LIBRARY(COREVIDEO_LIBRARY CoreVideo)
  FIND_LIBRARY(JAVAVM_LIBRARY JavaVM)
  SET(EXTRA_LIBS
    ${FOUNDATION_LIBRARY}
    ${AVFOUNDATION_LIBRARY}
    ${COREMEDIA_LIBRARY}
    ${COREVIDEO_LIBRARY}
    ${JAVAVM_LIBRARY}
  )
endif()
message (STATUS "EXTRA_LIBS=${EXTRA_LIBS}")

# Add JNI / Javah
create_javah(TARGET javah
  CLASSES ch.fhnw.ether.avion.Avion
  CLASSPATH ${CMAKE_SOURCE_DIR}/../src/main/java
  OUTPUT_DIR ${CMAKE_SOURCE_DIR}/generated
)
include_directories(${PROJECT_SOURCE_DIR}/generated)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/generated)

# Add avion include location
include_directories(${PROJECT_SOURCE_DIR}/avion)

# Add avion library
add_library(avion
  SHARED
  ${PROJECT_SOURCE_DIR}/avion/avion_jni.mm
  ${PROJECT_SOURCE_DIR}/avion/avion_avfoundation.mm
)
add_dependencies(avion javah)
target_link_libraries(avion ${EXTRA_LIBS})

if (APPLE)
  set_target_properties(avion PROPERTIES OUTPUT_NAME avion-darwin)
endif()


# Add avion test executable
add_executable(avion_test
  ${PROJECT_SOURCE_DIR}/avion_test/avion_test.mm
)
target_link_libraries(avion_test avion)

# Install avion library so it can be packaged by gradle build
install(TARGETS avion
  LIBRARY
  DESTINATION ${PROJECT_SOURCE_DIR}/../libs
)

# Finally, set build type (for use with make)
set(CMAKE_BUILD_TYPE Release)
