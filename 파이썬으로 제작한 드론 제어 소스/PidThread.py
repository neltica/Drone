import serial
import threading
import PidClass
import SensorThread
import time

class PidThread(threading.Thread):

	def __init__(self):
		threading.Thread.__init__(self)

		self.serialValue=serial.Serial('/dev/ttyACM0',9600)
		self.baseSpeed=100
		self.sensorValue=[0,0,0,0]
		self.sensorValueLock=threading.Lock()
		self.motorLock=threading.Lock()

		self.sensor=None

		self.stopFlag=False
		self.waitFlag=False
		self.motor1=PidClass.PidHoveringClass()
		self.motor2=PidClass.PidHoveringClass()
		self.motor3=PidClass.PidHoveringClass()
		self.motor4=PidClass.PidHoveringClass()
		self.motor1ControlValue=0
		self.motor2ControlValue=0
		self.motor3ControlValue=0
		self.motor4ControlValue=0

		self.cond=threading.Condition()

	
	def init(self,initTime=60):

		self.sensor=SensorThread.SensorThread(initTime)
		self.sensor.start()
	def run(self):

		while True:
			if self.stopFlag:
				break
			if self.waitFlag:
			
				self.wait()

			self.sensorValueLock.acquire()
			self.sensorValue=self.sensor.getSensorValue()

			motor1Roll=self.sensorValue[0]*-1
			motor3Roll=self.sensorValue[0]*-1
			motor3Pitch=self.sensorValue[1]*-1
			motor4Pitch=self.sensorValue[1]*-1

			self.sensorValueLock.release()
			
			self.motor1ControlValue=0
			self.motor2ControlValue=0
			self.motor3ControlValue=0
			self.motor4ControlValue=0

			self.motor1ControlValue=self.motor1.pidControl2(motor1Roll,self.sensorValue[1],self.sensorValue[2],self.sensorValue[3])
				
			self.motor2ControlValue=self.motor2.pidControl2(self.sensorValue[0],self.sensorValue[1],self.sensorValue[2],self.sensorValue[3])

			self.motor3ControlValue=self.motor3.pidControl2(motor3Roll,motor3Pitch,self.sensorValue[2],self.sensorValue[3])

			self.motor4ControlValue=self.motor4.pidControl2(self.sensorValue[0],motor4Pitch,self.sensorValue[2],self.sensorValue[3])

			self.motorLock.acquire()

			self.motor1ControlValue+=self.baseSpeed

			self.motor2ControlValue+=self.baseSpeed

			self.motor3ControlValue+=self.baseSpeed

			self.motor4ControlValue+=self.baseSpeed

			self.motorLock.release()

			if self.motor1ControlValue<100:
				self.motor1ControlValue=100
			elif self.motor1ControlValue>130:
				self.motor1ControlValue=130


			if self.motor2ControlValue<100:
				self.motor2ControlValue=100
			elif self.motor2ControlValue>130:
				self.motor2ControlValue=130


			if self.motor3ControlValue<100:
				self.motor3ControlValue=100
			elif self.motor3ControlValue>130:
				self.motor3ControlValue=130
			
			if self.motor4ControlValue<100:
				self.motor4ControlValue=100
			elif self.motor4ControlValue>130:
				self.motor4ControlValue=130


			self.motorLock.acquire()

#print "motor1:",self.motor1ControlValue," motor2:",self.motor2ControlValue," motor3:",self.motor3ControlValue," motor4:",self.motor4ControlValue
			self.serialValue.write(chr(self.motor1ControlValue))
			self.serialValue.write(chr(self.motor2ControlValue))
			self.serialValue.write(chr(self.motor3ControlValue))
			self.serialValue.write(chr(self.motor4ControlValue))
			self.serialValue.flushInput()	

			self.motorLock.release()

			

	def stop(self):
		print "pidThread stop"
		self.stopFlag=True
		self.setAllMotorSpeed100()
		self.sensor.stop()
		self.sensor.join()

	def setWait(self):
		self.waitFlag=True

	def wait(self):
		print "pidThread wait"
		self.sensor.setWait()

		self.cond.acquire()
		self.cond.wait()
		self.cond.release()



	def notify(self):
		print "pidThread notify"
		self.setAllMotorSpeed100()
		self.sensor.notify()

		self.cond.acquire()
		self.waitFlag=False
		self.cond.notify()
		self.cond.release()


	def getSensorValue(self):
		self.sensorValueLock.acquire()
		resultSensorValue=self.sensorValue
		self.sensorValueLock.release()
		return resultSensorValue

	def getMotorSpeed(self):
		self.motorLock.acquire()
		motor1=self.motor1ControlValue
		motor2=self.motor2ControlValue
		motor3=self.motor3ControlValue
		motor4=self.motor4ControlValue
		self.motorLock.release()
		return [motor1,motor2,motor3,motor4]

	def allUpMotor(self):
		print "all motor up"
		self.motorLock.acquire()
		self.baseSpeed+=1
		if self.baseSpeed>200:
			self.baseSpeed=200
		self.motorLock.release()

	def allDownMotor(self):
		print "all motor dowm"
		self.motorLock.acquire()
		self.baseSpeed-=1
		if self.baseSpeed<100:
			self.baseSpeed=100
		self.motorLock.release()

	def setAllMotorSpeed100(self):
		print "all motor set 100"
		self.motorLock.acquire()
		self.serialValue.write(chr(201))
		self.baseSpeed=100
		for i in range(0,16,1):
			self.serialValue.write(chr(100))
		self.serialValue.flushInput()
		self.motorLock.release()
