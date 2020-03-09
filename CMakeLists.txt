cmake_minimum_required(VERSION 3.5)
project(ontologenius_query)

function(add_onto_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()
    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()
    add_library(${TARGET} ${ARGN})
    target_include_directories(${TARGET} PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PRIVATE ${catkin_LIBRARIES})
    set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endfunction(add_onto_library)

function(add_onto_executable TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()
    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()
    add_executable(${TARGET} ${ARGN})
    add_dependencies(${TARGET} ontologenius_query_gencpp ${catkin_EXPORTED_TARGETS})
    target_include_directories(${TARGET} PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PRIVATE ${catkin_LIBRARIES})
    set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endfunction(add_onto_executable)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ontologenius
  roscpp
  std_msgs
  genmsg
  message_generation
)
find_package(ontologenius 0.2.5 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  OntologeniusQueryResponse.msg
)

add_service_files(
 FILES
 OntologeniusQueryService.srv
 OntologeniusQueryFullService.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES ontologenius_query
   CATKIN_DEPENDS ontologenius
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ontologenius_INCLUDE_DIRS}
)

add_onto_library(ontologenius_query_analyzer_lib
  src/VariableStorage.cpp
  src/QueryAnalyzer.cpp
  src/full/FullAnalyser.cpp
)

add_onto_executable(ontologenius_query src/nodes/single.cpp)
target_link_libraries(ontologenius_query PRIVATE ontologenius_query_analyzer_lib)

add_onto_executable(ontologenius_query_multi src/nodes/multi.cpp)
target_link_libraries(ontologenius_query_multi PRIVATE ontologenius_query_analyzer_lib)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ontologenius_query_tester test/ontologenius_query.test src/tests/ontologenius_query_test.cpp)
  set_target_properties(ontologenius_query_tester PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_include_directories(ontologenius_query_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_query_tester ontologenius_lib ${catkin_LIBRARIES})

endif()
