idf_component_register(
        SRCS main.cc
        INCLUDE_DIRS "")

# Per-board zone name, e.g.: idf.py -DEDGE_ZONE_NAME=entrance build
if(NOT DEFINED EDGE_ZONE_NAME)
        set(EDGE_ZONE_NAME "bedroom")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE EDGE_ZONE_NAME="${EDGE_ZONE_NAME}")

# 촬영용 완화 빌드: idf.py -DEDGE_DEMO=1 flash
# 되돌릴 때는 -DEDGE_DEMO=0 을 명시해야 한다. CMake 캐시에 값이 남으므로
# 그냥 idf.py build 하면 직전 설정이 그대로 유지된다.
if(NOT DEFINED EDGE_DEMO)
        set(EDGE_DEMO 0)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE EDGE_DEMO=${EDGE_DEMO})