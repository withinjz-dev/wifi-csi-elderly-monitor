idf_component_register(
        SRCS main.cc
        INCLUDE_DIRS "")

# Per-board zone name, e.g.: idf.py -DEDGE_ZONE_NAME=entrance build
if(NOT DEFINED EDGE_ZONE_NAME)
        set(EDGE_ZONE_NAME "bedroom")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE EDGE_ZONE_NAME="${EDGE_ZONE_NAME}")