#!/usr/bin/python

# Script to automate fusing rosinstall dependencies
# It takes two parameters: an origin file and a target file, both in yaml format
# Warning: the target file we be modified!
# Searches the target file for the dependecies enumerated by the origin file, and
# if they are not present, it adds them to the target file.

from optparse import OptionParser
import yaml
from pprint import pprint

def errorExit(str):
    print str
    exit(1)

# Rosinstall yaml file is a list of key:value pairs for the top level keys
# This function finds if the needle is already listed in the list of dependencies
def isListed(needle, haystacklist):
    found = False
    
    for listitem in haystacklist:
        for tpkey,tpdata in listitem.items():
            if tpdata["local-name"] == needle["local-name"]:
                found = True
                
    return found


# Get command line parameters
optparser = OptionParser()
optparser.add_option("-n", dest="newfile", help="New rosinstall file")
optparser.add_option("-t", dest="targetfile", help="Target rosinstall file")
optparser.add_option("-l", dest="listonly", help="Only list the changes")

(options, args) = optparser.parse_args()

#pprint(options)

targetfile = options.targetfile
newfile = options.newfile

# open the 
if newfile:
    newfileh = open(newfile, 'r')
else:
    errorExit("Newfile is missing.")

if targetfile:
    targetfileh = open(targetfile, 'r')
else:
    errorExit("Targetfile is missing.")
    
newfiledata = yaml.load(newfileh)
targetfiledata = yaml.load(targetfileh)

# close the files we already read
newfileh.close()
targetfileh.close()

# open the targetfile for writing
targetfileh = open(targetfile, 'a') 

# initialize list of dictionary entries for yaml. These are the ones that will need to be added
addlist = []

# Traverse all items in the newfile and check if they are present in the targetfile
# if they aren't, we need to add them to the targetfile list
for li in newfiledata:
    for k1,d1 in li.items():
        if not isListed(d1, targetfiledata):
            addlist.append({k1:d1})

# add the needed packets to the target file
targetfileh.write(yaml.dump(addlist, default_flow_style=False))
            
targetfileh.close()



