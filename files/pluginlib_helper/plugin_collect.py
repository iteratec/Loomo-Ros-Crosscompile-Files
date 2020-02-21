import string
import xml.etree.ElementTree as xml
from lxml import etree
from plugin_description import PluginDescription
import os, sys, inspect
import collections
import re


# I need rospkg. Tell me where it is!
import_folder = "files/rospkg/src/rospkg/"
DEBUG = False

# get the real path to the script in case it is a symlink
cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
if cmd_folder not in sys.path:
   sys.path.insert(0, cmd_folder)

# add folder to the path so we can import modules from it
cmd_subfolder = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile(inspect.currentframe()))[0], import_folder)))
if cmd_subfolder not in sys.path:
   sys.path.insert(0, cmd_subfolder)

from rospkg import RosPack


def test():
  root_scan_dir = "output"
  collect_plugins(root_scan_dir)

def collect_plugins(root_scan_dir):
  """
  Scan directories starting in the designated root to locate all
  packages that depend on pluginlib. This will indirectly tell us
  which packages potentially export plugins. Then we search for the
  plugin manifest file and we parse it to obtain all the exported
  plugin classes.
  root_scan_dir indicates the starting point for the scan
  returns the collected plugin classes
  """
  rp = RosPack([root_scan_dir])

  packages = rp.list()
  debug_print("Found packages:\n")
  #print packages
  debug_print()

  # Find all packages that depend on pluginlib and nodelet explicitely
  pluginlib_users = rp.get_depends_on('pluginlib', implicit=False)
  nodelet_users = rp.get_depends_on('nodelet', implicit=False)
  image_transport_users = rp.get_depends_on('image_transport', implicit=False)
  # Concatenate both lists removing the duplicates
  pluginlib_users += list(set(nodelet_users) - set(pluginlib_users))
  pluginlib_users += list(set(image_transport_users) - set(pluginlib_users))

  debug_print("Packages that depend on pluginlib:\n")
  debug_print(pluginlib_users)
  debug_print()

  # Within the packages that require pluginlib, search all their
  # dependencies for plugins
  plugin_classes = []
  for p in pluginlib_users:
    path = rp.get_path(p)
    debug_print(p + ": ")
    debug_print(path)
    exports = rp.get_manifest(p).exports
    debug_print("Exports: ")
    for e in exports:
      s = e.get("plugin")
      if s:
        s2 = string.replace(s, "${prefix}", path)
        debug_print(s2)
        f = open(s2, 'r')
        xml_str = f.read()
        xml_str = escape_xml(xml_str)
        plugin_classes += parse_plugin_xml(xml_str, p, path)
        #plugin_classes += parse_plugin_xml(s2)
        debug_print(plugin_classes)
        f.close()
    debug_print()
  return plugin_classes


def parse_plugin_xml(xml_str, package="", manifest=""):
  """
  Parse a plugin description and return all the plugin classes
  that were found.
  xml_str the xml string to parse.
  """
  plugin_classes = []
  parser = etree.XMLParser(recover=True)
  #xml_tree = xml.parse(file, parser=parser)
  #xml_root = xml_tree.getroot()
  xml_root = xml.fromstring(xml_str, parser)
  for xml_library in xml_root.iter("library"):
    debug_print(xml_library.attrib)
    for xml_class in xml_library.iter("class"):
      debug_print(xml_class.attrib)
      plugin_class = PluginDescription()
      try:
        plugin_class.base_class = xml_class.attrib["base_class_type"]
        plugin_class.library_name = xml_library.attrib["path"]
        plugin_class.class_type = xml_class.attrib["type"]
      except KeyError:
        debug_print("ERROR: NO BASE CLASS / PATH / TYPE!")
        exit(1)
      try:
        plugin_class.lookup_name = xml_class.attrib["name"]
      except KeyError:
        plugin_class.lookup_name = plugin_class.class_type

      desc = xml_class.find("description")
      if desc is not None:
        plugin_class.description = desc.text.strip()
      else:
        plugin_class.description = ""
      plugin_class.package = package
      plugin_class.manifest_path = manifest
      plugin_classes.append(plugin_class)
      debug_print(plugin_class)
  return plugin_classes

def escape_xml(str):
  """
  Some plugin description files are badly formatted so we need to
  parse strings and escape the < and > chars.
  """

  # Replace all non-printable characters by a space. This helps solve
  # some problems with newlines in the middle of a string.
  str = re.sub("\s+", " ", str)

  apostrophe_count = 0
  out = ""
  for c in str:
    if c == "\"":
      apostrophe_count += 1
      apostrophe_count = apostrophe_count % 2
    if c == "<" and apostrophe_count > 0:
      c = "&lt;"
    if c == ">" and apostrophe_count > 0:
      c = "&gt;"
    out += c
  return out

def debug_print(msg=""):
  if DEBUG:
    print msg

if __name__ == "__main__":
    test()
