find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)

add_library(FireBodyDragSystem SHARED
  src/BodyDragSystem.cc
)

target_include_directories(FireBodyDragSystem
  PRIVATE include
)

target_link_libraries(FireBodyDragSystem
  PRIVATE gz-sim8::gz-sim8
)

target_compile_options(FireBodyDragSystem
  PRIVATE -Wall -Wextra -Wpedantic
)

install(
  TARGETS FireBodyDragSystem
  LIBRARY DESTINATION lib
)
