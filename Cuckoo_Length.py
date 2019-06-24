from random import uniform
from random import randint 
import math
import numpy as np

def levyFlight(u):
	return math.pow(u,-1.0/3.0)

def randF():
	return uniform(0.0001,0.9999)

def calculateDistance(path,distanceMatrix):
        index = path[0]
        distance = 0
        for nextIndex in path[1:]:
                distance += distanceMatrix[index][nextIndex]
                index = nextIndex
        return distance+distanceMatrix[path[-1]][path[0]];

def swap(sequence,i,j):
        temp = sequence[i]
        sequence[i]=sequence[j]
        sequence[j]=temp

def twoOptMove(nest,a,c,m):
	nest = nest[0][:]
	swap(nest,a,c)
	return (nest,calculateDistance(nest,m))
	

def doubleBridgeMove(nest,a,b,c,d,x):
	nest = nest[0][:]
	swap(nest,a,b)
	swap(nest,b,d)
	return (nest , calculateDistance(nest,x))

def find_len(inputMatrix):
    n=int(np.sqrt(len(inputMatrix)))
    #n = 10
    inputMatrix = np.reshape(inputMatrix,(n,n))
    numNests = 10
    pa = int(0.2*numNests)
    pc = int(0.6*numNests)


    maxGen = 50

    
    nests = []
    initPath=[ix for ix in range(0,n)]  
   
    index = 0
    for i in range(numNests):
        if index == n-1:
            index = 0
        swap(initPath,index,index+1)
        index =index+1
        nests.append((initPath[:],calculateDistance(initPath,inputMatrix)))

    nests.sort(key=lambda tup: tup[1])

    for t in range(maxGen):
        cuckooNest = nests[randint(0,pc)]
        if(levyFlight(randF())>2):
            cuckooNest = doubleBridgeMove(cuckooNest,randint(0,n-1),randint(0,n-1),randint(0,n-1),randint(0,n-1),inputMatrix)
        else:
            cuckooNest = twoOptMove(cuckooNest,randint(0,n-1),randint(0,n-1),inputMatrix)
        randomNestIndex = randint(0,numNests-1)
        if(nests[randomNestIndex][1]>cuckooNest[1]):
            nests[randomNestIndex] = cuckooNest
        for i in range(numNests-pa,numNests):
            nests[i] = twoOptMove(nests[i],randint(0,n-1),randint(0,n-1),inputMatrix)
        nests.sort(key=lambda tup: tup[1])	
    return nests[0][1];

