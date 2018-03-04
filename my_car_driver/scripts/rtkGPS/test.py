
readfile = open("round4.txt",'r')
line = readfile.readline()
while line:
    temp = line.split(',')
    
    print float(temp[2])
    line = readfile.readline()


readfile.close()
