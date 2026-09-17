cmake_minimum_required(VERSION 3.16)
project(gz_lora_range_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(gz_lora_range_plugin SHARED src/LoRaRangeSystem.cpp)
target_link_libraries(gz_lora_range_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-math7::gz-math7
)

install(TARGETS gz_lora_range_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
