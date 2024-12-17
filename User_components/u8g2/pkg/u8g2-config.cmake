get_filename_component(_dir "u8g2-config.cmake" PATH)
get_filename_component(_prefix "${_dir}/../../.." ABSOLUTE)

include("${_prefix}/lib/cmake/u8g2/u8g2-targets.cmake")

set(u8g2_INCLUDE_DIRS "${_prefix}/include/u8g2")
