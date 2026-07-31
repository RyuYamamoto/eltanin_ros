# Requires 3.24 for the WHOLE_ARCHIVE link feature and LINK_LIBRARY_OVERRIDE_<item>. This bound is
# local to this nested project; the rest of the repository builds with 3.20. Jazzy / Noble ship
# 3.28, so the bound costs nothing today.
cmake_minimum_required(VERSION 3.24)
project(eltanin_link_check LANGUAGES CXX)

find_package(eltanin REQUIRED)

set(ELTANIN_MODULES core map map_io sensor planner control sim collision)

# One SHARED library per module, whole-archiving that module only.
#
# Linking a static archive into a shared object normally pulls in just the referenced members, so a
# non-PIC object can sit in the archive unnoticed; WHOLE_ARCHIVE forces every object through the
# link and turns a missing -fPIC into an error here instead of in some later node.
#
# Whole-archiving all eight modules into a single SHARED library does not work: eltanin::map depends
# publicly on eltanin::core, so core appears both directly and transitively and CMake rejects the
# mixed link feature; overriding the feature for every item then whole-archives libeltanin_core.a
# twice and the link fails on duplicate definitions. Keeping one whole-archived module per target
# means exactly one archive is expanded, and a failure names the offending module.
foreach(module IN LISTS ELTANIN_MODULES)
  add_library(link_check_${module} SHARED link_check.cpp)
  target_link_libraries(link_check_${module} PRIVATE eltanin::${module})
  set_property(TARGET link_check_${module}
    PROPERTY LINK_LIBRARY_OVERRIDE_eltanin::${module} WHOLE_ARCHIVE)
  # With every object pulled in, requiring a fully resolved link also checks that the
  # find_dependency(Eigen3) / find_dependency(yaml-cpp) calls in eltaninConfig.cmake and the
  # exported interface dependencies really are sufficient from a consumer's prefix.
  target_link_options(link_check_${module} PRIVATE -Wl,--no-undefined)
endforeach()
