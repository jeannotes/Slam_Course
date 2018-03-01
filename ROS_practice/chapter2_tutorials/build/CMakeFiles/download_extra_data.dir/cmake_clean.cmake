FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/chapter2_tutorials/msg"
  "../src/chapter2_tutorials/srv"
  "CMakeFiles/download_extra_data"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
