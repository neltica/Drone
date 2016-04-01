import smbus
import time

class MPU6050Class():
	
	def __init__(self,pwr1=0x6b,pwr2=0x6c):
		self.address=0x68
		self.pwr1=pwr1
		self.pwr2=pwr2
		self.accelScale=16384.0
		self.gyroScale=131
		self.radianTodegreeValue=180/3.14
		self.radianTodegreeValue=self.divideNumber(self.radianTodegreeValue)
		self.bus=smbus.SMBus(1)
		self.bus.write_byte_data(self.address,self.pwr2,0)

		self.preAccRoll=0.0
		self.preAccPitch=0.0
		self.preAccYaw=0.0
		self.preGyroRoll=0.0
		self.preGyroPitch=0.0
		self.preGyroYaw=0.0



	def read_byte(self,adr):
		return self.bus.read_byte_data(adr)
	
	def read_word(self,adr):
		high=self.bus.read_byte_data(self.address,adr)
		low=self.bus.read_byte_data(self.address,adr+1)
		val=(high<<8)+low
		return val

	def read_word_2c(self,adr):
		val=self.read_word(adr)
		if(val>=0x8000):
			return -((65535-val)+1)
		else:
			return val


	def printGyroAccel(self):
		gyroX=self.read_word_2c(0x43)
		gyroY=self.read_word_2c(0x45)
		gyroZ=self.read_word_2c(0x47)

		print "gyroX: ",gyroX," scaled: ",(gyroX/131)
		print "gyroY: ",gyroY," scaled: ",(gyroY/131)
		print "gyroZ: ",gyroZ," scaled: ",(gyroZ/131)

		accelX=self.read_word_2c(0x3b)
		accelY=self.read_word_2c(0x3d)
		accelZ=self.read_word_2c(0x3f)

		print "accelX: ",accelX," scaled: ",accelX/16384.0
		print "accelY: ",accelY," scaled: ",accelY/16384.0
		print "accelZ: ",accelZ," scaled: ",accelZ/16384.0

	def getAccel(self):
		accelX=self.read_word_2c(0x3b)
		accelY=self.read_word_2c(0x3d)
		accelZ=self.read_word_2c(0x3f)
		return (accelX,accelY,accelZ)

	def getGyro(self):
		gyroX=self.read_word_2c(0x43)
		gyroY=self.read_word_2c(0x45)
		gyroZ=self.read_word_2c(0x47)
		catchTime=time.time()
		return (gyroX,gyroY,gyroZ,catchTime)

	def getAccelRollPitchDegree(self):
		accelX=self.read_word_2c(0x3b)
		accelY=self.read_word_2c(0x3d)
		accelZ=self.read_word_2c(0x3f)
		accelX/=self.accelScale
		accelY/=self.accelScale
		accelZ/=self.accelScale
		accelX=self.divideNumber(accelX)
		accelY=self.divideNumber(accelY)
		accelZ=self.divideNumber(accelZ)
		if accelZ!=0:
			import math
			rollResult=math.atan(accelY/accelZ)
			rollResult=rollResult*self.radianTodegreeValue
			pitchResult=math.atan(accelX/accelZ)
			pitchResult=pitchResult*self.radianTodegreeValue
#print "Pitch degree: ",pitchResult	
#print "roll degree: ",rollResult
			self.preAccRoll=rollResult
			self.preAccPitch=pitchResult
			return (rollResult,pitchResult)
		else:
			return [self.preAccRoll,self.preAccPitch]


	def getGyroRollPitchYawDegree(self):
		gyroX=(self.read_word_2c(0x43)/self.gyroScale)
		gyroY=(self.read_word_2c(0x45)/self.gyroScale)
		gyroZ=(self.read_word_2c(0x47)/self.gyroScale)
		catchTime=time.time()

		gyroX=self.divideNumber(gyroX)
		gyroY=self.divideNumber(gyroY)
		gyroZ=self.divideNumber(gyroZ)

		return [gyroX,gyroY,gyroZ,catchTime]



	def getAccelRollDegree(self):
		accelY=self.divideNumber(self.read_word_2c(0x3d)/self.accelScale)
		accelZ=self.divideNumber(self.read_word_2c(0x3f)/self.accelScale)
		if accelZ!=0:
			import math
			rollResult=math.atan(accelY/accelZ)*self.radianTodegreeValue
			return rollResult
		else:
			return None




	def getAccelPitchDegree(self):
		accelX=self.divideNumber(self.read_word_2c(0x3b)/self.accelScale)
		accelZ=self.divideNumber(self.read_word_2c(0x3f)/self.accelScale)
		if accelZ!=0:
			import math
			pitchResult=math.atan(accelX/accelZ)*self.radianTodegreeValue
			return pitchResult

		else:
			return None

	def getGyroRollDegree(self):
		gyroX=self.read_word_2c(0x43)/self.gyroScale
		catchTime=time.time()
		return (gyroX,catchTime)

	def getGyroPitchDegree(self):
		gyroY=self.read_word_2c(0x45)/self.gyroScale
		catchTime=time.time()
		return (gyroY,catchTime)

	def getGyroYawDegree(self):
		gyroZ=self.read_word_2c(0x47)/self.gyroScale
		catchTime=time.time()
		return (gyroZ,catchTime)


	def divideNumber(self,number):
		number=round(number,2)
#print number
		return number


	
