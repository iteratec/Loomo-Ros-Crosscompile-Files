#!/usr/bin/env python

# Generates a helper file for the static version of ROS pluginlib.
#
# The function getStaticClassesAvailable is called by the pluginlib
# classLoader constructor to get the mapping between a library and it's
# exported plugin classes, as defined in the plugin description xml
# file. This is required in systems without access to the filesystem,
# e.g. Android.

from plugin_description import PluginDescription

tab = "  "

def main():
  file_name = "pluginlib_helper.cpp"
  fileh = open(file_name, "w")
  plugin1 = PluginDescription()
  plugin1.lookup_name = "name1"
  plugin1.class_type = "class1"
  plugin2 = PluginDescription()
  plugin2.lookup_name = "name2"
  plugin2.class_type = "class2"
  generate(fileh, [plugin1, plugin2])
  close(fileh)

#
# pluginClasses = {"library_path":library_path,
#                  "class_name":class_name,
#                  "class_type":class_type,
#                  "base_class":base_class,
#                  "class_description":class_description]
#
def generate(fileh, pluginClasses):
  putHeaderComment(fileh)
  fileh.write("\n")
  putIncludes(fileh)
  fileh.write("\n")
  fileh.write("using namespace pluginlib;\n\n")
  fileh.write("typedef std::map<std::string, ClassDesc> classes_available_map;\n")
  fileh.write("typedef std::pair<std::string, ClassDesc> plugin_pair;\n")
  fileh.write("classes_available_map getStaticClassesAvailable(void)\n{\n")
  fileh.write(tab + "classes_available_map pluginClasses;\n")
  # if there are plugin class definitions, populate the map
  if len(pluginClasses) > 0:
    for plugin in pluginClasses:
      fileh.write(tab + "pluginClasses.insert(\n")
      fileh.write(tab*2 + "plugin_pair(\"" + plugin.lookup_name + "\",\n")
      fileh.write(tab*3 + "ClassDesc(\"" + plugin.lookup_name + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"" + plugin.class_type + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"" + plugin.base_class + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"" + plugin.package + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"" + plugin.description + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"" + plugin.library_name + "\",\n")
      fileh.write(tab*3 + " "*10 + "\"\")));\n")
  fileh.write(tab + "return pluginClasses;\n")
  fileh.write("}")

# A comment explaining the file.
def putHeaderComment(fileh):
  fileh.write("""/**
 * This file was automatically generated.
 * Helper file for the static version of ROS pluginlib.
 * 
 * The function is called by the pluginlib classLoader constructor to
 * get the mapping between a library and it's exported plugin classes,
 * as defined in the plugin description xml file.
 * This is required in systems without access to the filesystem,
 * e.g. Android and the plugins are statically linked at build time.
 * 
 */\n""")

# Cpp includes
def putIncludes(fileh):
  fileh.write("""#include "boost/algorithm/string.hpp"
#include "class_loader/multi_library_class_loader.h"
#include <map>
#include <pluginlib/class_desc.h>\n""")


if (__name__ == "__main__"):
  main()
  
