cmake_minimum_required (VERSION 2.8)
project("utest")

#------------------------------------------------------------------------------
# Extract current version tag from git and set version variables
#------------------------------------------------------------------------------
execute_process(
    COMMAND git describe --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_HEAD_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    )

if (GIT_HEAD_TAG)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" GIT_VERSION ${GIT_HEAD_TAG})
    set(PROJECT_VERSION_STRING "${GIT_HEAD_TAG}")
else()
    set(PROJECT_VERSION_STRING "git")
endif()

add_definitions(-DVERSION=\"${PROJECT_VERSION_STRING}\")

if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake|MSBuild)")
    add_definitions("/W2")
else()
  #add_definitions("-Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-local-typedefs -Wno-unused-variable -Wno-unused-but-set-variable -Wno-strict-aliasing -pg")
  add_definitions("-Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-local-typedefs -Wno-unused-variable -Wno-unused-but-set-variable -Wno-strict-aliasing")
endif()

# ...add sources
file(GLOB APP_C_SRC
  "utest-common.c"
  "utest-mmngr.c"
  "utest-drm-display.c"
  "dl.c"
  "utest-vsink.c"
  "utest-vin.c"
  "utest-imr.c"
  "utest-png.c"
  "utest-app.c"
  "utest-mesh-generators.c"
  "utest-camera-math.c"
  "utest-yaml.c"
  "utest-main.c")

include_directories(${DRM_INCLUDE_DIR} ${GSTREAMER_INCLUDE_DIR} ${GLIB_INCLUDE_DIR} ${CAIRO_INCLUDE_DIR} ${YAML_INCLUDE_DIR})

add_executable(utest-cam-imr-drm ${APP_C_SRC})
target_link_libraries(utest-cam-imr-drm
  ${COMMON_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${WNDSYS_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${YAML_LIBRARIES}
  "png"
  "mmngr"
  "mmngrbuf"
  "z"
  "spnav"
  "input"
  )

set_target_properties(utest-cam-imr-drm PROPERTIES SKIP_BUILD_RPATH ON)

# ...installation
if (NOT BINARY_DIR)
  set(BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../output)
endif()

message(STATUS "Installation directory: ${BINARY_DIR}")

install(TARGETS utest-cam-imr-drm DESTINATION ${BINARY_DIR})
