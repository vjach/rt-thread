include(FindGit)
find_package(Git)

if (NOT Git_FOUND)
    message(FATAL_ERROR "Git not found!")
endif ()

include(FetchContent)

FetchContent_Declare(
    pico_sdk
    GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
    GIT_TAG 1.4.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk
    PATCH_COMMAND git am
    ${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-Weaken-panic-and-_exit-functions.patch
    ${CMAKE_CURRENT_SOURCE_DIR}/patches/0002-Set-XOSC-to-8-MHz.patch
    )
if(NOT pico_sdk__POPULATED)
    FetchContent_Populate(pico_sdk)
endif()

function(pico_message_debug MESSAGE)                                                                                                                                                                                           
  # The log-level system was added in CMake 3.15.                                                                                                                                                                            
  if(${CMAKE_VERSION} VERSION_LESS "3.15.0")                                                                                                                                                                                 
    message(${MESSAGE})                                                                                                                                                                                                    
  else()                                                                                                                                                                                                                     
    message(DEBUG ${MESSAGE})                                                                                                                                                                                              
  endif()                                                                                                                                                                                                                    
endfunction()

set (PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${pico_sdk_SOURCE_DIR}/tools/;${CMAKE_MODULE_PATH}")
find_package(Pioasm REQUIRED)
add_subdirectory("${pico_sdk_SOURCE_DIR}/tools")
