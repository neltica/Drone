class PidHoveringClass():
	def __init__(self):
		self.pGain=0
		self.iGain=0
		self.dGain=0

		self.dstRoll=0
		self.dstPitch=0
		self.dstYaw=0

		self.preErrorRoll=0
		self.preErrorPitch=0
		self.preErrorYaw=0


		self.pRoll=0
		self.iRoll=0
		self.dRoll=0


		self.pPitch=0
		self.iPitch=0
		self.dPitch=0

		self.pYaw=0
		self.iYaw=0
		self.dYaw=0


		self.dstValue=0
		self.preError=0
		self.i=0



	def __del__(self):
		pass

	def destValue(self,roll,pitch,yaw):
		self.dstRoll=roll
		self.dstPitch=pitch
		self.dstYaw=yaw

	def setGain(self,p,i,d):
		self.pGain=p
		self.iGain=i
		self.dGain=d
		
		self.pRoll=0
		self.iRoll=0
		self.dRoll=0


		self.pPitch=0
		self.iPitch=0
		self.dPitch=0

		self.pYaw=0
		self.iYaw=0
		self.dYaw=0

		self.preError=0
		self.i=0




	def pidControl2(self,roll,pitch,yaw,samplingTime):
		absRoll=abs(roll)
		absPitch=abs(pitch)

		max=absRoll

		if absPitch>max:
			max=absPitch

		if (max==absRoll) or (max==absRoll*-1):
			max=roll
		elif (max==absPitch) or (max==absPitch*-1):
			max=pitch


		error=self.dstValue-max

		p=error
		self.i+=error*samplingTime
		d=(error-self.preError)*samplingTime

		self.preError=error

		result=p*self.pGain+self.i*self.iGain+d*self.dGain

		return int(result)






