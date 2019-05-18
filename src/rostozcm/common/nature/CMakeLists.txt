set(nature_src ${nature_src} angle.cpp)
set(nature_src ${nature_src} timestamp.cpp)
add_library(nature ${nature_src})

add_executable(angle_test angle_test.cpp)
target_link_libraries(angle_test nature)
