import PidThread
import ServerForJAVA
import time




def stateStringMake():
	dataForSend=str(motorValue[0])+","+str(motorValue[1])+","+str(motorValue[2])+","+str(motorValue[3])+","+str(sensorValue[0])+","+str(sensorValue[1])+","+str(sensorValue[2])+"\n"
	return dataForSend

def setPidGain(compareData):
	pidThread.motor1.pGain=float(compareData[1])
	pidThread.motor1.iGain=float(compareData[2])
	pidThread.motor1.dGain=float(compareData[3])

	pidThread.motor2.pGain=float(compareData[4])
	pidThread.motor2.iGain=float(compareData[5])
	pidThread.motor2.dGain=float(compareData[6])

	pidThread.motor3.pGain=float(compareData[7])
	pidThread.motor3.iGain=float(compareData[8])
	pidThread.motor3.dGain=float(compareData[9])

	pidThread.motor4.pGain=float(compareData[10])
	pidThread.motor4.iGain=float(compareData[11])
	pidThread.motor4.dGain=float(compareData[12])

pidThread=PidThread.PidThread()
pidThread.init(5)

print "server Start"
server=ServerForJAVA.ServerSocketForJAVA()
print "connect"
pidThread.start()
sensorValue=pidThread.getSensorValue()
motorValue=pidThread.getMotorSpeed()
dataForSend=str(motorValue[0])+","+str(motorValue[1])+","+str(motorValue[2])+","+str(motorValue[3])+","+str(sensorValue[0])+","+str(sensorValue[1])+","+str(sensorValue[2])+"\n"


server.send(dataForSend)



while True:
	try:

		dataForRecv=server.recv()
		
		if not dataForRecv:
			pass
		else:
			tempdata=dataForRecv.split("\n")
			compareData=tempdata[0].split(',')
			
			if compareData[0]=="state":

				sensorValue=pidThread.getSensorValue()
				motorValue=pidThread.getMotorSpeed()
				dataForSend=stateStringMake()
			
			elif compareData[0]=="pidget":
				pidThread.setWait()
				dataForSend=str(pidThread.motor1.pGain)+","+str(pidThread.motor1.iGain)+","+str(pidThread.motor1.dGain)+","+str(pidThread.motor2.pGain)+","+str(pidThread.motor2.iGain)+","+str(pidThread.motor2.dGain)+","+str(pidThread.motor3.pGain)+","+str(pidThread.motor3.iGain)+","+str(pidThread.motor3.dGain)+","+str(pidThread.motor4.pGain)+","+str(pidThread.motor4.iGain)+","+str(pidThread.motor4.dGain)+"\n"
			elif compareData[0]=="pidset":
				setPidGain(compareData)
				dataForSend=str(pidThread.motor1.pGain)+","+str(pidThread.motor1.iGain)+","+str(pidThread.motor1.dGain)+","+str(pidThread.motor2.pGain)+","+str(pidThread.motor2.iGain)+","+str(pidThread.motor2.dGain)+","+str(pidThread.motor3.pGain)+","+str(pidThread.motor3.iGain)+","+str(pidThread.motor3.dGain)+","+str(pidThread.motor4.pGain)+","+str(pidThread.motor4.iGain)+","+str(pidThread.motor4.dGain)+"\n"
				pidThread.notify()
			elif compareData[0]=="up":
				pidThread.allUpMotor()		
				
				sensorValue=pidThread.getSensorValue()
				motorValue=pidThread.getMotorSpeed()
				dataForSend=stateStringMake()
				pass
			elif compareData[0]=="down":
				pidThread.allDownMotor()

				sensorValue=pidThread.getSensorValue()
				motorValue=pidThread.getMotorSpeed()
				dataForSend=stateStringMake()
				pass
			elif compareData[0]=="shutdown":
				pidThread.setWait()
				allMotorStop()
				sensorValue=pidThread.getSensorValue()
				motorValue=pidThread.getMotorSpeed()
				dataForSend=stateStringMake()

			elif compareData[0]=="restart":
				pidThread.notify()
				sensorValue=pidThread.getSensorValue()
				motorValue=pidThread.getMotorSpeed()
				dataForSend=stateStringMake()
			
			elif compareData[0]=="quit":
				pidThread.stop()
				pidThread.join()
				print "Quadcopter End"
				break

			server.send(dataForSend)
	except:
		pidThread.stop()
		pidThread.join()
