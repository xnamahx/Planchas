cmake_minimum_required(VERSION 3.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-pretarget.cmake)

include_directories( 
	"${C74_INCLUDES}"
)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

set(DATTORRO_SRC
       src/Plateau/Dattorro.cpp
       src/Common/DSP/LinearEnvelope.cpp
       src/Common/DSP/OnePoleFilters.cpp
       )

add_library(LIBDATORRO ${DATTORRO_SRC} )

add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} LIBDATORRO)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-posttarget.cmake)
