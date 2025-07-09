# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kir/esp/esp-idf/components/bootloader/subproject"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/tmp"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/src/bootloader-stamp"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/src"
  "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kir/Documents/embedde_testing/embedded_task/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
