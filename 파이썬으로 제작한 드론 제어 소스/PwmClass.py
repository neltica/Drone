from RPIO import PWM
import os
class PwmClass():

	def __init__(self):
		self.servo=PWM.Servo()
		self.us=10
		self.width17=100
		self.width23=100
		self.width25=100
		self.width16=100
		self.motor_width=[[17,self.width17],[23,self.width23],[25,self.width25],[16,self.width16]]
		for i in self.motor_width:
			self.servo.set_servo(i[0],self.us*i[1])


	def __del__(self):
		for i in self.motor_width:
			self.servo.stop_servo(i[0])

	def setMotorSpeed(self,motorNum,pwrNum):
		self.motor_width[motorNum][1]=pwrNum
		if self.motor_width[motorNum][1]>200:
			self.motor_width[motorNum][1]=200
		elif self.motor_width[motorNum][1]<100:
			self.motor_width[motorNum][1]=100
		self.servo.set_servo(self.motor_width[motorNum][0],self.us*self.motor_width[motorNum][1])


	def setMotorSpeedForPid(self,motorNum,pwrNum):
		tempSpeed=self.motor_width[motorNum][1]+pwrNum
#		print "motor",motorNum,"sum: ",tempSpeed
		if tempSpeed>200:
			tempSpeed=200
		elif tempSpeed<100:
			tempSpeed=100
		self.servo.set_servo(self.motor_width[motorNum][0],self.us*tempSpeed)

	def upMotor(self,motorNum,pwrNum):
		if self.motor_width[motorNum][1]<200:
			self.motor_width[motorNum][1]+=1
			self.servo.set_servo(self.motor_width[0],self.us*self.motor_width[1])

	def downMotor(self,motorNum,pwrNum):
		if self.motor_width[motorNum][1]>100:
			self.motor_width[motorNum][1]-=1
			self.servo.set_servo(self.motor_width[0],self.us*self.motor_width[1])


	def allUpMotor(self):
		flag=0
		for i in range(0,4,1):
			if self.motor_width[i][1]>=200:
				flag=1
				break
		if flag==0:
		 	for i in range(0,4,1):
				self.motor_width[i][1]+=1

		for i in self.motor_width:
			self.servo.set_servo(i[0],self.us*i[1])

	def allDownMotor(self):
		flag=0

		for i in range(0,4,1):
			if self.motor_width[i][1]<=100:
				flag=1
				break

		if flag==0:
			for i in range(0,4,1):
				self.motor_width[i][1]-=1

		for i in self.motor_width:
			self.servo.set_servo(i[0],self.us*i[1])

	def allMotorStop(self):
		for i in self.motor_width:
			self.servo.stop_servo(i[0])
