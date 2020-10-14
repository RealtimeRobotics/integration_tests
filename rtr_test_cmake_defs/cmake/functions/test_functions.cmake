# A simple function for defining tests with no parent library
# You must have defined GTEST_COMPILE_OPTIONS in the parent scope
# Can pass additional libraries to link to directly (i.e. Threads::Threads)
function(rs_add_gtest TestName)
  catkin_add_gtest(${TestName}
    tests/${TestName}.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  )
  target_compile_options(${TestName} PRIVATE ${GTEST_COMPILE_OPTIONS})
  target_link_libraries(${TestName} ${catkin_LIBRARIES} ${ARGN})
  set_property(TARGET ${TestName} PROPERTY POSITION_INDEPENDENT_CODE ON)
endfunction()

# A simple function for defining rostests with no parent library
function(rs_add_rostest TestName)
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${TestName} tests/${TestName}.test tests/${TestName}.cpp)
    target_link_libraries(${TestName} ${catkin_LIBRARIES})
    set_property(TARGET ${TestName} PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_compile_options(${TestName} PRIVATE ${GTEST_COMPILE_OPTIONS})
  endif()
endfunction()
