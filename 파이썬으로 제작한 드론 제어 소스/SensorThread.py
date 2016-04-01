import complementaryFilter
import threading
import time
class SensorThread(threading.Thread):

	def __init__(self,initTime=60):
		threading.Thread.__init__(self)
		self.complementary=complementaryFilter.ComplementaryFilterClass()
		self.sensorLock=threading.Lock()
		self.cond=threading.Condition()
		self.sensorValue=0
		self.stopFlag=False
		self.waitFlag=False
		print "sensor init"
		self.complementary.initTime=initTime
		self.complementary.initGyroSensor()
		print "sensor init complete"


	def run(self):
		while True:
			if self.stopFlag:
				break
			if self.waitFlag:
				self.wait()
			self.sensorLock.acquire()
			self.sensorValue=self.complementary.getComplementary()
			self.sensorLock.release()
	def getSensorValue(self):
		self.sensorLock.acquire()
		resultSensorValue=self.sensorValue
		self.sensorLock.release()
		return self.sensorValue

	def stop(self):
		print "Sensor Thread stop"
		self.stopFlag=True

	def setWait(self):
		self.waitFlag=True
	def wait(self):
		print "Sensor Thread wait"
		self.cond.acquire()
		self.cond.wait()
		self.cond.release()

	def notify(self):
		print "Sensor Thread notify"
		self.cond.acquire()
		self.waitFlag=False
		print "sensor re init start"
		self.complementary.initGyroSensor()
		print "sensor re init complete"
		self.cond.notify()
		self.cond.release()
