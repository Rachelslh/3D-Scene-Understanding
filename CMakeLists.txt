cmake_minimum_required(VERSION 3.0.0)
project(testC VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(include modules/preprocessing/libfreenect2_grabber.cpp
            modules/Depth-Cut_Based_Ground_Detection/include/ground_search.cpp)
add_executable(Main modules/main.cpp ${include})

if( PCL_FOUND AND OpenCV_FOUND AND freenect2_FOUND)

    include_directories("modules")

    # [C/C++]>[General]>[Additional Include Directories]
    include_directories( $ENV{HOME}/freenect2/include )
    include_directories( ${PCL_INCLUDE_DIRS} )
    include_directories( ${OpenCV_INCLUDE_DIRS} )

    # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
    add_definitions( ${PCL_DEFINITIONS} )

    # For Use Not PreCompiled Features
    # add_definitions( -DPCL_NO_PRECOMPILE )

    # [Linker]>[General]>[Additional Library Directories]
    link_directories( ${PCL_LIBRARY_DIRS} )

    # [Linker]>[Input]>[Additional Dependencies]
    target_link_libraries( Main ${freenect2_LIBRARIES} )
    target_link_libraries( Main ${PCL_LIBRARIES} )
    target_link_libraries( Main ${OpenCV_LIBS} )

endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
