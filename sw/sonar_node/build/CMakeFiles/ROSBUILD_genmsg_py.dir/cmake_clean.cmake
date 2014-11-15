FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sonar_node/msg"
  "../src/sonar_node/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/sonar_node/msg/__init__.py"
  "../src/sonar_node/msg/_Pinger.py"
  "../src/sonar_node/msg/_PingerArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
