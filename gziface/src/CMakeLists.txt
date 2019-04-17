# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

ign_add_component(gziface SOURCES ${sources} GET_TARGET_NAME gziface_target)

target_link_libraries(${gziface_target}
  PUBLIC
  sdformat${SDFORMAT_VER}::sdformat${SDFORMAT_VER})

# Build the unit tests.
ign_build_tests(TYPE UNIT
    SOURCES ${gtest_sources}
    LIB_DEPS ${gziface_target} sdformat${SDFORMAT_VER}::sdformat${SDFORMAT_VER})

