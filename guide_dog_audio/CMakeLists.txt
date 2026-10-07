cmake_minimum_required(VERSION 3.8)
project(guide_dog_audio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(guide_dog_interfaces REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# Only the runtime library is installed on the Jetson (no libespeak-ng-dev),
# so link the versioned .so directly; tts_node.cpp declares the API it uses.
find_library(ESPEAK_NG_LIBRARY NAMES espeak-ng libespeak-ng.so.1)
if(NOT ESPEAK_NG_LIBRARY)
  message(FATAL_ERROR "libespeak-ng not found (apt package libespeak-ng1)")
endif()

add_executable(tts_node src/tts_node.cpp)

ament_target_dependencies(tts_node
  rclcpp
  ament_index_cpp
  std_msgs
  guide_dog_interfaces
  unitree_api
)

target_link_libraries(tts_node
  ${ESPEAK_NG_LIBRARY}
  nlohmann_json::nlohmann_json
  OpenSSL::Crypto
)

install(TARGETS
  tts_node
  DESTINATION lib/${PROJECT_NAME}
)

# go2_rtc_keepalive: the Go2 speaker cuts clips short unless a WebRTC session is open.
# gtts_synth.py: Google TTS for tts_node, run under a Python that has gTTS.
install(PROGRAMS
  scripts/go2_rtc_keepalive
  scripts/go2_rtc_keepalive.py
  scripts/gtts_synth.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
