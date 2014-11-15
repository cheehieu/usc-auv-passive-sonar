FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sonar_node/msg"
  "../src/sonar_node/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Pinger.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Pinger.lisp"
  "../msg_gen/lisp/PingerArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_PingerArray.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
