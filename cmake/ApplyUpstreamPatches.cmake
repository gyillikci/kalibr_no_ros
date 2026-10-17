# Copy the immutable upstream snapshot into the build tree and apply the
# reviewed portability patches from patches/*.patch in lexical order.
#
# The copy is refreshed only when the snapshot digest or a patch changes, so
# incremental builds keep their object files.
function(kalibr_apply_upstream_patches pristine destination patch_dir)
  file(GLOB patches LIST_DIRECTORIES false "${patch_dir}/*.patch")
  list(SORT patches)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${patch_dir}" "${pristine}.sha256" ${patches})

  file(READ "${pristine}.sha256" stamp)
  foreach(patch IN LISTS patches)
    file(SHA256 "${patch}" digest)
    get_filename_component(name "${patch}" NAME)
    string(APPEND stamp "${name} ${digest}\n")
  endforeach()

  set(stamp_file "${destination}.stamp")
  if(EXISTS "${stamp_file}")
    file(READ "${stamp_file}" previous)
    if(previous STREQUAL stamp)
      return()
    endif()
  endif()

  # find_program(... REQUIRED) needs CMake 3.18; Ubuntu 20.04 ships 3.16.
  find_program(KALIBR_PATCH_EXECUTABLE patch)
  if(NOT KALIBR_PATCH_EXECUTABLE)
    message(FATAL_ERROR "The 'patch' program is required to prepare upstream Kalibr")
  endif()
  message(STATUS "Preparing patched upstream Kalibr in ${destination}")
  file(REMOVE_RECURSE "${destination}")
  get_filename_component(parent "${destination}" DIRECTORY)
  file(MAKE_DIRECTORY "${parent}")
  file(COPY "${pristine}" DESTINATION "${parent}")
  foreach(patch IN LISTS patches)
    get_filename_component(name "${patch}" NAME)
    execute_process(
      COMMAND "${KALIBR_PATCH_EXECUTABLE}" -p1 --forward --batch --silent
        --no-backup-if-mismatch -i "${patch}"
      WORKING_DIRECTORY "${destination}"
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
      message(FATAL_ERROR "Failed to apply ${name}:\n${output}")
    endif()
    message(STATUS "  applied ${name}")
  endforeach()
  file(WRITE "${stamp_file}" "${stamp}")
endfunction()
