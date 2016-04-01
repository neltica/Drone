import socket
import time

server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.bind(('',6000))
server.listen(1)

connection,address=server.accept()


import I2cClass
i2c=I2cClass.MPU6050Class()

while True:
	rollDegree,pitchDegree=i2c.getAccelRollPitchDegree()
	print "rollDegree: ",rollDegree,"\npitchDegree: ",pitchDegree
	if rollDegree!=None and pitchDegree!=None:
		data=str(rollDegree)+","+str(pitchDegree)

		recvData=connection.recv(1024)
		connection.send(data)

connection.close()
server.close()
