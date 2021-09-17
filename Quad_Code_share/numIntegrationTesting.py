# numIntegration testing

import numIntegration
import bufferShawn

timeStep = 1
yLine = [0 ,1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
yPrbla = [0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100]
inputBufferLine = bufferShawn.bufferShawn(3)
inputBufferPrbla = bufferShawn.bufferShawn(3)
areaLine = 0
areaPrbla = 0
evenAreaPrbla = 0
oddAreaPrbla = 0

for i in range(0,len(yLine)):
	inputBufferLine.put(yLine[i])
	inputBufferPrbla.put(yPrbla[i])
	areaLine = areaLine + numIntegration.numIntegration("Trap",timeStep,inputBufferLine)
	areaPrbla = areaPrbla + numIntegration.numIntegration("Trap",timeStep,inputBufferPrbla)
	#print(areaLine)  # should be 9.5 (not quite keeping up by using the midpoints)
	#print(areaPrbla)  # should be 335
	if((-1.0)**i >=0):
		#print("even")
		evenAreaPrbla = evenAreaPrbla + numIntegration.numIntegration("Simp",timeStep,inputBufferPrbla)
		print(evenAreaPrbla) # should be 333.33333
	else:
		#print("odd")
		oddAreaPrbla = oddAreaPrbla + numIntegration.numIntegration("Simp",timeStep,inputBufferPrbla)
		print(oddAreaPrbla) # should be 333.33333





