cmake_minimum_required(VERSION 3.20)
project(dual_tuner_recorder LANGUAGES C VERSION 1.0.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LibSDRplay REQUIRED)

if (NOT LIBSDRPLAY_FOUND)
    message(FATAL_ERROR "SDRPlay development files not found...")
endif ()
message(STATUS "LIBSDRPLAY_INCLUDE_DIRS - ${LIBSDRPLAY_INCLUDE_DIRS}")
message(STATUS "LIBSDRPLAY_LIBRARIES - ${LIBSDRPLAY_LIBRARIES}")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(SOURCE_FILES dual_tuner_recorder.c)
include_directories(${LIBSDRPLAY_INCLUDE_DIRS})

add_executable(dual_tuner_recorder ${SOURCE_FILES})
if (WIN32)
    set(PTHREAD_LIBRARY libwinpthread.a)
endif ()
target_link_libraries(dual_tuner_recorder ${LIBSDRPLAY_LIBRARIES} ${PTHREAD_LIBRARY})
