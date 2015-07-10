project(ProstheticEye)

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

find_package(CGAL QUIET COMPONENTS Core)
find_package(PCL 1.3 REQUIRED common io visualization surface features search filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  include( CGAL_CreateSingleSourceCGALProgram )

  include_directories (BEFORE "/usr/local/include/CGAL-4.4/include")
  
  SET(CMAKE_CXX_FLAGS "-std=c++0x -Wcpp")

  create_single_source_cgal_program( "deluanay_graph.cpp" )
  target_link_libraries(deluanay_graph ${PCL_LIBRARIES})
  create_single_source_cgal_program( "symmetry.cpp" )
  target_link_libraries(symmetry ${PCL_LIBRARIES})
  create_single_source_cgal_program( "ui.cpp" )
  target_link_libraries(ui ${PCL_LIBRARIES})

else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()
