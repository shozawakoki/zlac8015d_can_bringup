cmake_minimum_required(VERSION 3.8)
project(zlac_can_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# With colcon --symlink-install, installing PROGRAMS can point at the
# non-executable GitHub source (mode 0644). Make executable copies in the
# build tree so the ROS 2 libexec symlinks have valid execute permissions.
set(_zlac_script_dir "${CMAKE_CURRENT_BINARY_DIR}/executable_scripts")
file(MAKE_DIRECTORY "${_zlac_script_dir}")
foreach(_zlac_script record_experiment.py run_gain_sweep.py)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_zlac_script}"
    "${_zlac_script_dir}/${_zlac_script}"
    COPYONLY
  )
  file(CHMOD "${_zlac_script_dir}/${_zlac_script}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE)
endforeach()

install(
  PROGRAMS
    "${_zlac_script_dir}/record_experiment.py"
    "${_zlac_script_dir}/run_gain_sweep.py"
  DESTINATION lib/${PROJECT_NAME}
)

# With --symlink-install, the Python entry point may execute from its
# build-tree target. Place sweep_core.py beside it in both locations:
# build/executable_scripts/ (symlink target) and install/libexec/ (ROS 2).
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/sweep_core.py"
  "${_zlac_script_dir}/sweep_core.py"
  COPYONLY
)
install(
  FILES "${_zlac_script_dir}/sweep_core.py"
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
