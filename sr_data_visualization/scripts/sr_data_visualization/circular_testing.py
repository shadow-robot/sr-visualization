import numpy as np
a = []

xlim = 10
n = np.linspace(0, xlim - 1, xlim)
y = []
y= ((n * 0.0) + 50)
print "y", y
print "n", n

addedDataArray = []
addedDataArray.append(1)
addedDataArray.append(2)
addedDataArray.append(3)
addedDataArray.append(4)
addedDataArray.append(5)
addedDataArray.append(6)
addedDataArray.append(7)
print "addedDataArray", addedDataArray

while len(addedDataArray) > 0:
    print "len(addedDataArray)", len(addedDataArray)
    y = np.roll(y, -1)
    print "y", y
    y[-1] = addedDataArray[0]
    print "y", y
    del (addedDataArray[0])
    print "addedDataArray", addedDataArray

print "addedDataArray", addedDataArray
print "y", y

i = 0
while i < 30:
    addedDataArray.append(i)
    i += 1

print addedDataArray
del addedDataArray[0:20]
print addedDataArray