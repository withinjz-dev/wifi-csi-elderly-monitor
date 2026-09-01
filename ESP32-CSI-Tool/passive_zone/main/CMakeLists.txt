idf_component_register(
        SRCS main.cc
        INCLUDE_DIRS "")

if(NOT DEFINED EDGE_ZONE_NAME)
        set(EDGE_ZONE_NAME "entrance")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE EDGE_ZONE_NAME="${EDGE_ZONE_NAME}")
