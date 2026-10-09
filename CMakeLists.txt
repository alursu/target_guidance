cmake_minimum_required(VERSION 3.10)
project(tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Guide (тепловизионная USB-камера, свой SDK): подключается АВТОМАТИЧЕСКИ,
# если рядом с проектом лежит папка camera_sdk (include/guidescusb2.h + lib/libGuideSCUSB2.so).
# Выключить: -DGUIDE_CAMERA=OFF; путь указать: -DGUIDE_CAMERA_SDK=<путь>.
option(GUIDE_CAMERA "Использовать камеру Guide (SDK libGuideSCUSB2)" ON)
set(GUIDE_CAMERA_SDK "${CMAKE_CURRENT_SOURCE_DIR}/camera_sdk" CACHE PATH
    "Путь к папке camera_sdk камеры Guide")

set(TRK_USE_GUIDE OFF)
if(GUIDE_CAMERA AND EXISTS "${GUIDE_CAMERA_SDK}/include/guidescusb2.h"
   AND EXISTS "${GUIDE_CAMERA_SDK}/lib/libGuideSCUSB2.so")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(TRK_USE_GUIDE ON)
        message(STATUS "Guide camera SDK: ${GUIDE_CAMERA_SDK}")
    else()
        message(STATUS "Guide camera SDK найден, но .so — arm64, а сборка идёт на "
                       "${CMAKE_SYSTEM_PROCESSOR} — Guide-код выключен (проверка на Pi)")
    endif()
else()
    message(STATUS "Guide camera SDK НЕ найден (${GUIDE_CAMERA_SDK}) — сборка только V4L2")
endif()

add_executable(tracker
    main.cpp
    crsf_bridge.cpp
    egomotion.cpp
    tracking.cpp
    guidance.cpp
    osd.cpp
    warn.cpp
    framesource.cpp
)

if(TRK_USE_GUIDE)
    target_compile_definitions(tracker PRIVATE TRK_GUIDE_CAMERA)
    target_include_directories(tracker PRIVATE ${GUIDE_CAMERA_SDK}/include)
    target_link_libraries(tracker PRIVATE
        ${GUIDE_CAMERA_SDK}/lib/libGuideSCUSB2.so
        usb-1.0 udev m)
endif()

target_include_directories(tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracker PRIVATE ${OpenCV_LIBS} Threads::Threads)