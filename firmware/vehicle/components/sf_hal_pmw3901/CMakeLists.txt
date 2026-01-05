cmake_minimum_required(VERSION 3.5)

# Register component
idf_component_register(
    SRCS "src/pmw3901.c" "src/pmw3901_wrapper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
