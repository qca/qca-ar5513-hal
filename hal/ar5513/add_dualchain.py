#!/usr/bin/env python

"add_dualchain.py -- Add Dual Chain Support to AR5513 INI file"

import sys

# print usage and exit
def usage():
    print "usage:  add_dualchain inFile outFile"
    sys.exit(1)

# filefind
def convert_ini(inFile,outFile):

    # can we open file?
    try:
        inf = open(inFile, 'r')
    # if not, exit
    except:
        print inFile, ":", sys.exc_info()[1]
        usage()

    # can we open output file
    try:
        outf = open(outFile,'w')
    # if not, exit
    except:
        print outFile, ":", sys.exc_info()[1]
        usage()
        
    allLines = inf.readlines()

    for eachLine in allLines:
        newLine = eachLine.lstrip()
        if len(newLine) != 0:
            if newLine[0] == '{':
                tokens = newLine.split(',')
                regStr = tokens[0][2:]
                regAddr = int(regStr,16)

                if (regAddr >= 0x9880 and regAddr <= 0x98fc):
                    regAddr = (regAddr & ~0xf000) | 0xb000

                regStr = tokens[0][0:1] + " 0x%08x" % regAddr

                tokens[0] = regStr

                newLine = ""
                for tok in tokens:
                    newLine = newLine + tok + ','
                    
                outf.write(newLine[:len(newLine)-1])
            else:
                outf.write(eachLine)
        else:
            outf.write(eachLine)
            
# validate arguments and calls filefind()
def checkargs():

    # check args; 'argv' comes from 'sys' module
    argc = len(sys.argv)
    if argc != 3:
        usage()

    # call add_dualchain.filefind() with args
    convert_ini(sys.argv[1],sys.argv[2])
    
if __name__ == '__main__':
    print 'Starting add_dualchain'
    checkargs()
