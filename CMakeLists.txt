cmake_minimum_required (VERSION 2.8)
project(cam-imr-drm)

# ...some external headers having no corresponding "FindXXX.cmake" facilities
SET(DRM_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/drm")
SET(GSTREAMER_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/gstreamer-1.0" "${CMAKE_SYSROOT}/usr/lib/gstreamer-1.0/include")
SET(GLIB_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/glib-2.0" "${CMAKE_SYSROOT}/usr/lib/glib-2.0/include" "${CMAKE_SYSROOT}/usr/lib64/glib-2.0/include" "/usr/lib/x86_64-linux-gnu/glib-2.0/include")
SET(CAIRO_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/cairo" "${CMAKE_SYSROOT}/usr/include/pango-1.0")

# ...common libraries
SET(COMMON_LIBRARIES "pthread" "dl" "z" "m")
SET(GLIB_LIBRARIES "gobject-2.0" "glib-2.0" "gmodule-2.0" "gthread-2.0")
SET(GSTREAMER_LIBRARIES "gstreamer-1.0" "gstrtp-1.0" "gstpbutils-1.0" "gstbase-1.0" "gstapp-1.0" "gstvideo-1.0" "gstallocators-1.0" )
SET(CAIRO_LIBRARIES "cairo"  "pango-1.0" "pangocairo-1.0")
SET(MMNGR_LIBRARIES "mmngr" "mmngrbuf")
SET(WNDSYS_LIBRARIES "drm" "udev")

# ...force stripping of the output files during installation
SET(CPACK_STRIP_FILES TRUE)

# ...unit-test application
add_subdirectory("utest")
