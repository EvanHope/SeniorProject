# test the filter functions
import bufferShawn
import filterShawn

fc = 5
fs = 100
order = 1

print(fc)
print(fs)

# run these only once to calculate the coefficients
coeff = filterShawn.butterworth(order,"HPF",fc,fs)
#print(coeff)

inputList = [1,1,1,1,1,1,1,1,1,1]
outputBuffer = bufferShawn.bufferShawn(order)
inputBuffer = bufferShawn.bufferShawn(order+1)


for i in range(0,len(inputList)):
	inputBuffer.put(inputList[i])
	outputBuffer.put(filterShawn.filterEval(coeff,order,inputBuffer,outputBuffer))
	print(outputBuffer.prev(0))














