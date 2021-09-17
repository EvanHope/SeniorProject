# create a simple buffer data structure
# operation is like a fixed length que which also allows access to prior
# buffer items

class bufferShawn:
	
	def __init__(self,size):
		self.items = [0.0 for i in range(0,size)]
	
	def put(self,item):
		for i in range(self.size()-1,0,-1):
			self.items[i] = self.items[i-1]
		self.items[0] = item
	
	def prev(self,index):
		return self.items[index]
	
	def size(self):
		return len(self.items)










