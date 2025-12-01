set(COMMON_SRC_FILES 
  "${CMAKE_SOURCE_DIR}/common/src/vector.cpp"
  "${CMAKE_SOURCE_DIR}/common/src/rayos.cpp"
)

set(CURRENT_DIR_SRC_FILES 
  "${CMAKE_CURRENT_SOURCE_DIR}/test_vector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_color.cpp"  
  "${CMAKE_CURRENT_SOURCE_DIR}/test_ppm_writer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_rayos.cpp"
)

add_unit_test_target(
  TARGET_NAME utcommon
  SOURCE_FILES ${COMMON_SRC_FILES} ${CURRENT_DIR_SRC_FILES}
  LIBRARY_FILTER common
  COVERAGE_DIR coverage-common
  LIBRARY_TO_LINK common
)


add_test(NAME utcommon COMMAND $<TARGET_FILE:utcommon>)


target_include_directories(utcommon PRIVATE
  ${CMAKE_SOURCE_DIR}/aos/src
  ${CMAKE_SOURCE_DIR}/soa/src
)