cmake_minimum_required(VERSION 3.22.1)
project(tita_system_interfaces LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
)

set(SRV_FILES
        "srv/PowerStateSetSrv.srv"
        "srv/PowerSelfTestSrv.srv"
        "srv/PowerHeartBeatSrv.srv"
        "srv/FanModeSetSrv.srv"
        "srv/HeadLightControlSrv.srv"
        "srv/TailLightControlSrv.srv"
        "srv/LegLightControlSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        ${SRV_FILES}
        DEPENDENCIES std_srvs diagnostic_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
