FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/chapter2_tutorials/msg"
  "../src/chapter2_tutorials/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/chapter2_tutorials/srv/__init__.py"
  "../src/chapter2_tutorials/srv/_chapter2_srv1.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
