FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sonar_node/msg"
  "../src/sonar_node/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/sonar_node.dir/src/cheetah.o"
  "CMakeFiles/sonar_node.dir/src/sonar_fft.o"
  "CMakeFiles/sonar_node.dir/src/sonar_utils.o"
  "CMakeFiles/sonar_node.dir/src/sonar_driver.o"
  "../bin/sonar_node.pdb"
  "../bin/sonar_node"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang C CXX)
  INCLUDE(CMakeFiles/sonar_node.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
