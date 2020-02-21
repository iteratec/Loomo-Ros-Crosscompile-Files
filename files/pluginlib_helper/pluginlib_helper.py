#!/usr/bin/env python

import os, sys, inspect
import argparse
import plugin_collect
import pluginlib_helper_gen

# get the real path to the script in case it is a symlink
cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
if cmd_folder not in sys.path:
   sys.path.insert(0, cmd_folder)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("-scanroot", nargs="?", default="", help="Root folder to start scanning for plugins.")
  parser.add_argument("-cppout", nargs="?", default="", help="Name of the cpp output file that will contain the static plugin loader code.")
  parser.add_argument("-buildfile", nargs="?", default="", help="Name of the build file.")
  
  args = parser.parse_args()
  
  print "scanroot: %s" % args.scanroot
  print "cppout: %s" % args.cppout
  print "buildfile: %s" % args.buildfile
  
  plugin_classes = plugin_collect.collect_plugins(args.scanroot)
  
  for p in plugin_classes:
    print p
  
  print "Found %d plugin classes." % len(plugin_classes)
  print "Generating cpp helper module..."
  
  fileh = open(args.cppout, "w")
  pluginlib_helper_gen.generate(fileh, plugin_classes)
  fileh.close()

if (__name__ == "__main__"):
  main()
