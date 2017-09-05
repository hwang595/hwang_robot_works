import re

d = open('E:\CS799\RawData\dataRevised\\referenceData.dat', 'r')
data = d.readlines()
outData = []
outlines = []
for dataline in data:
    sear =  re.search('mx = (.+) my = (.+) mz = (.+) mG', dataline)
    if sear != None:
        for i in range(3):
            outlines.append(sear.groups()[i])
        outData.append(outlines)
    outlines = []
d.close()

with open('E:\CS799\RawData\dataRevised\mref.dat', 'w') as wrFile:
    for i in range(len(outData)):
        outLine = outData[i]
        for num in outLine:
            wrFile.write(num)
            wrFile.write(' ')
        wrFile.write('\r')
        print('line %s is done!' % str(i))
        
        
