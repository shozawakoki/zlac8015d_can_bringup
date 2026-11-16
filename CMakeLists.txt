cmake_minimum_required(VERSION 3.8)
project(zlac_can_bringup)

find_package(ament_cmake REQUIRED)

# Only resources used by zlac_can_bringup.launch.py.
install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install a self-contained ROS 2 executable and its pure-Python helpers.
# With --symlink-install, GitHub source mode 0644 is not executable;
# create an executable build-tree copy before installing into libexec.
set(_trial_script_dir "${CMAKE_CURRENT_BINARY_DIR}/trial_scripts")
file(MAKE_DIRECTORY "${_trial_script_dir}")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_cmdvel_trial.py"
  "${_trial_script_dir}/run_cmdvel_trial.py"
  COPYONLY
)
file(CHMOD "${_trial_script_dir}/run_cmdvel_trial.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmdvel_trial_core.py"
  "${_trial_script_dir}/cmdvel_trial_core.py"
  COPYONLY
)
install(
  PROGRAMS "${_trial_script_dir}/run_cmdvel_trial.py"
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES "${_trial_script_dir}/cmdvel_trial_core.py"
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
