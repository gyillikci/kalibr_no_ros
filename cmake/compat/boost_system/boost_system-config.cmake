# Boost 1.89 removed the boost_system stub library and its CMake package;
# Boost.System has been header-only since 1.69. Upstream Kalibr still lists
# "system" in every find_package(Boost COMPONENTS ...), so provide an empty
# interface target that satisfies those requests on newer Boost releases.
if(NOT TARGET Boost::system)
  add_library(Boost::system INTERFACE IMPORTED)
  if(TARGET Boost::headers)
    set_target_properties(Boost::system PROPERTIES
      INTERFACE_LINK_LIBRARIES Boost::headers)
  endif()
endif()
set(boost_system_FOUND TRUE)
