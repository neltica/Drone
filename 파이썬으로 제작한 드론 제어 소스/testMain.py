import PwmClass
import ServerForJAVA
import time

pwm=PwmClass.PwmClass()

server=ServerForJAVA.ServerSocketForJAVA()


print "connect"

while True:
	try:
		print "recv"
		data=server.recv()

		splitData=data.split('\n')
		splitData=splitData[0].split(',')
		if splitData[0]=="getMotorSpeed":
			m1Speed=pwm.us*pwm.motor_width[0][1]
			m2Speed=pwm.us*pwm.motor_width[1][1]
			m3Speed=pwm.us*pwm.motor_width[2][1]
			m4Speed=pwm.us*pwm.motor_width[3][1]
			print "send"
			server.send("ms,"+str(m1Speed)+","+str(m2Speed)+","+str(m3Speed)+","+str(m4Speed)+"\n")

		elif splitData[0]=="mu":
			pwm.allUpMotor()
			m1Speed=pwm.us*pwm.motor_width[0][1]
			m2Speed=pwm.us*pwm.motor_width[1][1]
			m3Speed=pwm.us*pwm.motor_width[2][1]
			m4Speed=pwm.us*pwm.motor_width[3][1]
			print "send"
			server.send("ms,"+str(m1Speed)+","+str(m2Speed)+","+str(m3Speed)+","+str(m4Speed)+"\n")



		
		elif splitData[0]=="md":
			pwm.allDownMotor()
			m1Speed=pwm.us*pwm.motor_width[0][1]
			m2Speed=pwm.us*pwm.motor_width[1][1]
			m3Speed=pwm.us*pwm.motor_width[2][1]
			m4Speed=pwm.us*pwm.motor_width[3][1]
			print "send"
			server.send("ms,"+str(m2Speed)+","+str(m2Speed)+","+str(m3Speed)+","+str(m4Speed)+"\n")

	except:
		print "except!!!!"
		pass
	
	time.sleep(0.1)
