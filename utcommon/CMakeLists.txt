# utcommon/CMakeLists.txt

# 1. real code we want to test
set(COMMON_SRC_FILES
  "${CMAKE_SOURCE_DIR}/common/src/vector.cpp"
  "${CMAKE_SOURCE_DIR}/common/src/camera.cpp"   # you need this for make_camera_from_config
)

# 2. your tests
set(CURRENT_DIR_SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/test_camera.cpp"
)

# 3. create a UNIQUE test target name (not "utcommon")
add_unit_test_target(
  TARGET_NAME utcommon_camera      # <<< changed from "utcommon"
  SOURCE_FILES
    ${COMMON_SRC_FILES}
    ${CURRENT_DIR_SRC_FILES}
  LIBRARY_FILTER common
  COVERAGE_DIR coverage-common
  LIBRARY_TO_LINK common
)

# 4. make sure gtest headers are visible
if (TARGET GTest::gtest)
  target_include_directories(utcommon_camera PRIVATE
    $<TARGET_PROPERTY:GTest::gtest,INTERFACE_INCLUDE_DIRECTORIES>
  )
endif ()
