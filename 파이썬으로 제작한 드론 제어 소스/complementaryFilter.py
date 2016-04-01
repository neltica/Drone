import I2cClass
import time
import os

class ComplementaryFilterClass():
	def __init__(self):
		self.initFlag=False
		self.i2c=None
		self.biasX=0
		self.biasY=0
		self.biasZ=0

		self.kpX=10
		self.kpY=10
		self.kpZ=10

		self.kiX=3
		self.kiY=3
		self.kiZ=3


		self.sumX=0.0
		self.sumY=0.0
		self.sumZ=0.0

		self.wIcX=0.001
		self.wIcY=0.001
		self.wIcZ=0.001

		self.gapTime=0
		self.sumTime=0
		self.preTime=0
		self.initTime=60


		self.preX=0.0
		self.preY=0.0
		self.preZ=0.0

		self.wPreIX=0
		self.wPreIY=0
		self.wPreIZ=0

		self.preFilterAngleX=0
		self.preFilterAngleY=0
		self.preFilterAngleZ=0

		self.preAccelAngleX=0.0
		self.preAccelAngleY=0.0
		self.preAccelAngleZ=0.0

		self.intTemp1X=0
		self.intTemp1Y=0
		self.intTemp1Z=0

		self.preTemp1X=0
		self.preTemp1Y=0
		self.preTemp1Z=0


		self.preTemp2X=0
		self.preTemp2Y=0
		self.preTemp2Z=0


		self.filterAngleX=0
		self.filterAngleY=0
		self.filterAngleZ=0





	
	def initGyroSensor(self):
		self.i2c=I2cClass.MPU6050Class()
		startTime=time.time()
		count=0
		i=0
		while True:
			nowTime=time.time()-startTime
			if nowTime>self.initTime:
				count=i-1
				break;
			gyro=self.i2c.getGyroRollPitchYawDegree()
			print "gyro init :",gyro
			gyro=list(gyro)
			self.biasX+=gyro[0]
			self.biasY+=gyro[1]
			self.biasZ+=gyro[2]
			i+=1
		
		self.biasX/=count
		self.biasY/=count
		self.biasZ/=count
	
		
		self.initFlag=True



	def getGyroHDRsadariIntegral2(self):



		gyro=self.i2c.getGyroRollPitchYawDegree()
		gyro[0]-=self.biasX
		gyro[1]-=self.biasY
		gyro[2]-=self.biasZ

	
		self.gapTime=gyro[3]-self.preTime


		gyro[0]+=self.wPreIX
		gyro[1]+=self.wPreIY
		gyro[2]+=self.wPreIZ

		self.sumX+=(gyro[0]+self.preX)*self.gapTime/2
		self.sumY+=(gyro[1]+self.preY)*self.gapTime/2
		self.sumZ+=(gyro[2]+self.preZ)*self.gapTime/2
	


		def sign(x):
			if x>0:
				return 1
			elif x==0:
				return 0
			else:
				return -1
	

		self.wPreIX-=sign(self.sumX)*self.wIcX
		self.wPreIY-=sign(self.sumY)*self.wIcY
		self.wPreIZ-=sign(self.sumZ)*self.wIcZ

		self.preX=gyro[0]
		self.preY=gyro[1]
		self.preZ=gyro[2]
		self.preTime=gyro[3]
		self.sumTime+=self.gapTime

		return [self.sumX,self.sumY,self.sumZ]


	def complementaryFilter(self,accel,gyroHDR):
		

		temp1X=self.preFilterAngleX-self.preAccelAngleX
		temp1Y=self.preFilterAngleY-self.preAccelAngleY
	
		self.intTemp1X=self.intTemp1X+(temp1X+self.preTemp1X)*self.gapTime/2
		self.intTemp1Y=self.intTemp1Y+(temp1Y+self.preTemp1Y)*self.gapTime/2
	
		temp2X=(temp1X*-self.kpX)+(self.intTemp1X*-self.kiX)+gyroHDR[0]
		temp2Y=(temp1Y*-self.kpY)+(self.intTemp1Y*-self.kiY)+gyroHDR[1]
	
		self.filterAngleX+=((temp2X+self.preTemp2X)*self.gapTime/2)
		self.filterAngleY+=((temp2Y+self.preTemp2Y)*self.gapTime/2)
		self.filterAngleZ=gyroHDR[2]

		self.preTemp1X=temp1X
		self.preTemp1Y=temp1Y
	
		self.preTemp2X=temp2X
		self.preTemp2Y=temp2Y
	
	
		self.preFilterAngleX=self.filterAngleX
		self.preFilterAngleY=self.filterAngleY
		self.preAccelAngleX,self.preAccelAngleY=accel
	
		return [int(self.filterAngleX),int(self.filterAngleY),int(self.filterAngleZ),self.gapTime]


	def printComplementary(self):
		if self.initFlag==False:
			self.initGyroSensor()
		

		self.preTime=time.time()

		while True:
			accel=self.i2c.getAccelRollPitchDegree()
			print "accel:",accel
			gyro=self.getGyroHDRsadariIntegral2()
			complementary=self.complementaryFilter(accel,gyro)
			print "biasX:",self.biasX," biasY:",self.biasY," biasZ:",self.biasZ
			print "roll:",complementary[0],"\npitch:",complementary[1],"\nyaw:",complementary[2],"\ntime:",self.sumTime

			time.sleep(0.01)
			os.system('clear')


	def getComplementary(self):
		if self.initFlag==False:
			self.initGyroSensor()

		if self.preTime==0:
			self.preTime=time.time()
		accel=self.i2c.getAccelRollPitchDegree()
		gyro=self.getGyroHDRsadariIntegral2()
		complementary=self.complementaryFilter(accel,gyro)

		return complementary
