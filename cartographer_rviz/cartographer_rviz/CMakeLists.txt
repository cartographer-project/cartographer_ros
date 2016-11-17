google_library(drawable_submap
  USES_CARTOGRAPHER
  USES_EIGEN
  SRCS
    drawable_submap.cc
  HDRS
    drawable_submap.h
)

google_library(submaps_display
  USES_CARTOGRAPHER
  SRCS
    submaps_display.cc
  HDRS
    submaps_display.h
  DEPENDS
    drawable_submap
)
