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
    PATCH_COMMAND git am ${CMAKE_CURRENT_SOURCE_DIR}/patches
    )
if(NOT pico_sdk__POPULATED)
    FetchContent_Populate(pico_sdk)
endif()

