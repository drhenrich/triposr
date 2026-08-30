# Nur fuer die PlatformIO-Umgebung `usb` (framework = arduino, espidf).
# Die Umgebung `wifi` baut ohne CMake und ignoriert diese Datei.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lidar3d)
