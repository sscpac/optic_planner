ament_add_gtest(optic_test optic_test.cpp)
target_link_libraries(optic_test ${PROJECT_NAME} dl)
target_compile_definitions(optic_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")