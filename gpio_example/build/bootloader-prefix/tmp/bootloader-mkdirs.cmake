# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/julian-sanchez/esp/esp-idf/components/bootloader/subproject"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/tmp"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/src/bootloader-stamp"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/src"
  "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/julian-sanchez/Universidad/Sistemas_embebidos_avanzados/gpio_example/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
