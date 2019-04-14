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
find_package(ontologenius 0.2.0 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
 FILES
 OntologeniusQueryService.srv
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
  ${ontoloGenius_INCLUDE_DIRS}
)

add_onto_library(ontologenius_query_analyzer_lib
  src/VariableStorage.cpp
  src/QueryAnalyzer.cpp
)

add_onto_executable(ontologenius_query src/nodes/single.cpp)
target_link_libraries(ontologenius_query PRIVATE ontologenius_query_analyzer_lib)

add_onto_executable(ontologenius_query_multi src/nodes/multi.cpp)
target_link_libraries(ontologenius_query_multi PRIVATE ontologenius_query_analyzer_lib)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ontologenius_query.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
