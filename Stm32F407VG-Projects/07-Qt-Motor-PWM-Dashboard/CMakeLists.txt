cmake_minimum_required(VERSION 3.16)

project(MotorPWMDashboard VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialPort)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        circulargauge.cpp
        circulargauge.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(MotorPWMDashboard
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(MotorPWMDashboard
        ${PROJECT_SOURCES}
    )
endif()

target_link_libraries(MotorPWMDashboard PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::SerialPort)

# Windows specific to hide console window in release mode
set_target_properties(MotorPWMDashboard PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
