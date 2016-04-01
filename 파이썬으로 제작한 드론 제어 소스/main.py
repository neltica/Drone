"""
I2cClass is I2c protocol Class, this is connect and recv Degree Data from MPU6050 Gyro/Accel Sensor
PwmClass is motor class, bldc motor needs PWM protocol and this class is available that.
PidClass is Hovering Class that is available to quardcopter is hover in sky.
ServerForJAVA Class need for communication. because between Java and Python has different data type.
"""

import I2cClass
import PwmClass
import PidClass
import ServerForJAVA

i2c=I2cClass.MPU6050Class()
pwm=PwmClass.PwmClass()

while True:
	firstDegree=i2c.getAccelRollPitchDegree()
	if firstDegree[0]!=None and firstDegree[1]!=None:
		break


motor1Pid=PidClass.PidHoveringClass(firstDegree[0],firstDegree[1])
motor2Pid=PidClass.PidHoveringClass(firstDegree[0],firstDegree[1])
motor3Pid=PidClass.PidHoveringClass(firstDegree[0],firstDegree[1])
motor4Pid=PidClass.PidHoveringClass(firstDegree[0],firstDegree[1])

motor1Pid.pGain=1
motor2Pid.pGain=1
motor3Pid.pGain=1
motor4Pid.pGain=1

import threading

def run():
	while True:
		roll,pitch=i2c.getAccelRollPitchDegree()
		motor1Pid.setInputFilter(roll*-1,pitch)
		motor2Pid.setInputFilter(roll,pitch)
		motor3Pid.setInputFilter(roll*-1,pitch*-1)
		motor4Pid.setInputFilter(roll,pitch*-1)

		resultRoll1,resultPitch1=motor1Pid.pController()
		resultRoll2,resultPitch2=motor2Pid.pController()
		resultRoll3,resultPitch3=motor3Pid.pController()
		resultRoll4,resultPitch4=motor4Pid.pController()
		
		if resultRoll1>resultPitch1:
			pwm.setMotorSpeed(0,int(resultRoll1))
		elif resultRoll1<resultPitch1:
			pwm.setMotorSpeed(0,int(resultPitch1))

		if resultRoll2>resultPitch2:
			pwm.setMotorSpeed(1,int(resultRoll2))
		elif resultRoll2<resultPitch2:
			pwm.setMotorSpeed(1,int(resultPitch2))
		
		if resultRoll3>resultPitch3:
			pwm.setMotorSpeed(2,int(resultRoll3))
		elif resultRoll3<resultPitch3:
			pwm.setMotorSpeed(2,int(resultPitch3))

		if resultRoll4>resultPitch4:
			pwm.setMotorSpeed(3,int(resultRoll4))
		elif resultRoll4<resultPitch4:
			pwm.setMotorSpeed(3,int(resultPitch4))
		

thread=threading.Thread(target=run,args=())
#thread.start()


server=ServerForJAVA.ServerSocketForJAVA()
print "connect"

while True:
	data=server.recv()
	splitData=data.split('\n')
	splitData=splitData[0].split(',')
	if splitData[0]=="getMotorSpeed":
		m1Speed=pwm.us*pwm.motor_width[0][1]
		m2Speed=pwm.us*pwm.motor_width[1][1]
		m3Speed=pwm.us*pwm.motor_width[2][1]
		m4Speed=pwm.us*pwm.motor_width[3][1]
		server.send("ms,"+m1Speed+","+m2Speed+","+m3Speed+","+m4Speed+"\n")
		pass
	elif splitData[0]=="mu" or splitData[0]=="md":
		pwm.motor_width[0][1]+=int(splitData[1])
		pwm.motor_width[1][1]+=int(splitData[1])
		pwm.motor_width[2][1]+=int(splitData[1])
		pwm.motor_width[3][1]+=int(splitData[1])
		m1Speed=pwm.us*pwm.motor_width[0][1]
		m2Speed=pwm.us*pwm.motor_width[1][1]
		m3Speed=pwm.us*pwm.motor_width[2][1]
		m4Speed=pwm.us*pwm.motor_width[3][1]
		server.send("ms,"+m1Speed+","+m2Speed+","+m3Speed+","+m4Speed+"\n")
       		pass
