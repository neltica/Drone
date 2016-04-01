package com.example.controller2;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class StateGetClass extends Thread {

	private DataInputStream dataInputStream;
	private DataOutputStream dataOutputStream;
	private String receiveDataToString;
	private byte[] sendBuffer;
	
	private ControlActivity parent; 
	
	
	private boolean waitFlag,stopFlag,waitCheckFlag;

	public StateGetClass(ControlActivity parent) {
		waitFlag=false;
		stopFlag=false;
		sendBuffer=new String("state\n").getBytes();
		this.parent=parent;
	}
	
	public void setWait()
	{
		waitFlag=true;
	}
	
	private synchronized void waitting()
	{
		try {
			wait();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public synchronized void setNotify()
	{
		waitFlag=false;
		notify();
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		super.run();

//		try {
//			Thread.sleep(100);
//		} catch (InterruptedException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}
		
		try {
			dataInputStream = new DataInputStream(SocketClass.socket.getInputStream());
			dataOutputStream = new DataOutputStream(SocketClass.socket.getOutputStream());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		while (true) {
			
			if(stopFlag)
			{
				break;
			}
			if(waitFlag)
			{
				waitCheckFlag=true;
				waitting();
				waitCheckFlag=false;
			}
			
			try {
				// Log.i("send", new String(sendBuffer));
				dataOutputStream.write(sendBuffer);
				
				byte[] receiveData = new byte[1024];
				
				Log.i("recv", "recv");
				dataInputStream.read(receiveData);

				receiveDataToString = new String(receiveData).split("\n")[0];
				
				String [] recvData=receiveDataToString.split(",");
				
				Datas.motor1Speed=recvData[0];
				Datas.motor2Speed=recvData[1];
				Datas.motor3Speed=recvData[2];
				Datas.motor4Speed=recvData[3];
				Datas.rollValue=recvData[4];
				Datas.pitchValue=recvData[5];
				Datas.yawValue=recvData[6];
				
				//setMotorSensorValue();
				handler.sendEmptyMessage(0);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			
			try {
				Thread.sleep(Datas.serverSamplingTime);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
	}
	
	Handler handler = new Handler()
	{
		public void handleMessage(final Message msg)
		{
			setMotorSensorValue();
		}
	};
	
	private void setMotorSensorValue()
	{
		parent.setMotorSensorValue();
	}
	
	public boolean getWaitCheckFlag()
	{
		return waitCheckFlag;
	}
}



