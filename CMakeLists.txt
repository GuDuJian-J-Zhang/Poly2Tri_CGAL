cmake_minimum_required(VERSION 3.10)

set(ProjectName Poly2Tri)

# set the project name
project(${ProjectName} 
        VERSION 1.0)

message("WordDir: "  ${CMAKE_CURRENT_SOURCE_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
#target_include_directories(${ProjectName} PUBLIC ${SOURCE_DIR}/inc)
set(3RDS ${CMAKE_CURRENT_SOURCE_DIR}/3rds)

include_directories(${INC_DIR})
include_directories(${INC_DIR}/cgal/Polygon/include)
include_directories(${INC_DIR}/cgal/Triangulation_2/include)
include_directories(${INC_DIR}/cgal/Kernel_23/include)
include_directories(${INC_DIR}/cgal/Installation/include)
include_directories(${INC_DIR}/cgal/STL_Extension/include)
include_directories(${INC_DIR}/cgal/Circulator/include)
include_directories(${INC_DIR}/cgal/Cartesian_kernel/include)
include_directories(${INC_DIR}/cgal/Filtered_kernel/include)
include_directories(${INC_DIR}/cgal/Number_types/include)
include_directories(${INC_DIR}/cgal/TDS_2/include)
include_directories(${INC_DIR}/cgal/Profiling_tools/include)
include_directories(${INC_DIR}/cgal/Stream_support/include)
include_directories(${INC_DIR}/cgal/Algebraic_foundations/include)
include_directories(${INC_DIR}/cgal/Interval_support/include)
include_directories(${INC_DIR}/cgal/Modular_arithmetic/include)
include_directories(${INC_DIR}/cgal/Distance_2/include)
include_directories(${INC_DIR}/cgal/Distance_3/include)
include_directories(${INC_DIR}/cgal/Intersections_2/include)
include_directories(${INC_DIR}/cgal/Intersections_3/include)
include_directories(${INC_DIR}/cgal/Homogeneous_kernel/include)
include_directories(${INC_DIR}/cgal/Kernel_d/include)
include_directories(${INC_DIR}/cgal/Arithmetic_kernel/include)
include_directories(${INC_DIR}/cgal/Hash_map/include)
include_directories(${INC_DIR}/cgal/Spatial_sorting/include)
include_directories(${INC_DIR}/cgal/Property_map/include)

# add boost
set(BOOST_DIR "C:/gudujian/boost_1_82_0/boost_1_82_0")
include_directories(${BOOST_DIR})
# end add boost

file(GLOB_RECURSE SOURCES 
    ${SOURCE_DIR}/*.cpp 
    ${SOURCE_DIR}/*.c
    ${SOURCE_DIR}/*.h 
    ${INC_DIR}/*.h
)

option(BUILD_SHARED_LIBS FALSE "If TRUE will build as shared lib")
if (BUILD_SHARED_LIBS)
    add_definitions( -DPOLY2TRI_BUILD_DLL )
    add_library(${ProjectName} SHARED ${SOURCES})
else()
    add_definitions( -DPOLY2TRI_BUILD_AS_EXECUTABLE )
    add_executable(${ProjectName} ${SOURCES})
endif()