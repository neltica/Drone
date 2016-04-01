import socket

class ServerSocketForJAVA():
	def __init__(self,ip='',port=6000,bufferSize=1024):
		self.bufferSize=bufferSize
		self.adr=(ip,port)
		self.server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		self.server.bind(self.adr)
		self.server.listen(1)
		self.connection,self.address=self.server.accept()

	def __del__(self):
		print "Server del"
		self.connection.close()
		self.server.close()
		pass


	def send(self,data):
		self.connection.send(data)

	def recv(self):

		data=self.connection.recv(self.bufferSize)
		return data	

	def closeServer(self):
		self.connection.close()


	def openServer(self):

		self.connection,self.address=self.server.accept()
		
