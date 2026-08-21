find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)

add_library(FireMotorCommandMuxSystem SHARED
  src/MotorCommandMuxSystem.cc
)

target_include_directories(FireMotorCommandMuxSystem
  PRIVATE include
)

target_link_libraries(FireMotorCommandMuxSystem
  PRIVATE gz-sim8::gz-sim8
)

target_compile_options(FireMotorCommandMuxSystem
  PRIVATE -Wall -Wextra -Wpedantic
)

install(
  TARGETS FireMotorCommandMuxSystem
  LIBRARY DESTINATION lib
)
