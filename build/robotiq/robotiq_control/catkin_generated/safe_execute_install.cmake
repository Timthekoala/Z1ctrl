execute_process(COMMAND "/home/timothy/Z1fr/build/robotiq/robotiq_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/timothy/Z1fr/build/robotiq/robotiq_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
