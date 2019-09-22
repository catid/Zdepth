cmake_minimum_required(VERSION 3.5)
project(codecs LANGUAGES CXX)


################################################################################
# Dependencies

if (NOT TARGET nvcuvid)
    add_subdirectory(nvcuvid nvcuvid)
endif()


################################################################################
# Source

set(INCLUDE_FILES
    include/VideoCodec.hpp
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/VideoCodec.cpp
)

include_directories(include ${CUDA_INCLUDE_DIR})


################################################################################
# Targets

# codecs library

add_library(codecs STATIC ${SOURCE_FILES})
target_include_directories(codecs PUBLIC include ${CUDA_INCLUDE_DIR})
target_link_libraries(codecs PUBLIC
    nvcuvid
)

install(FILES ${INCLUDE_FILES} DESTINATION include)
install(TARGETS codecs DESTINATION lib)
