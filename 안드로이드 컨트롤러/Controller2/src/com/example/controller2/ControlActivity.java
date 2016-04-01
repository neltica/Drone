package com.example.controller2;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.lang.Thread.State;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.ToggleButton;

public class ControlActivity extends Activity implements Runnable,OnClickListener{

	//private ToggleButton serverOnOffButton;
	
	private Button upButton,downButton,rightButton,leftButton,quitButton,restartButton,stateButton;
	
	private TextView motor1,motor2,motor3,motor4,roll,pitch,yaw,ipInfo;
	
	private Thread thread;

	private int flag;
	
	private DataInputStream dataInputStream;
	
	private DataOutputStream dataOutputStream;
	
	public String receiveDataToString;
	
	private byte[] sendBuffer;
	
	private StateGetClass stateGetClass;
	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// TODO Auto-generated method stub
		
		menu.add(0,1,0,"setting");
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// TODO Auto-generated method stub
		
		switch(item.getItemId())
		{
		case 1:
			stateGetClass.setWait();
			while(true)
			{
				if(stateGetClass.getWaitCheckFlag())
				{
					break;
				}
			}
			Intent intent=new Intent(getApplicationContext(),PidSetActivity.class);
			startActivity(intent);
		}
		return super.onOptionsItemSelected(item);
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		
		Log.i("controlactivity", "create");
		setContentView(R.layout.activity_control);
				
		Datas.totalSpeed=0;
		
		//serverOnOffButton=(ToggleButton)findViewById(R.id.toggleButton1);
		//serverOnOffButton.setChecked(true);
		
		upButton=(Button)findViewById(R.id.button1);
		leftButton=(Button)findViewById(R.id.button2);
		downButton=(Button)findViewById(R.id.button3);
		rightButton=(Button)findViewById(R.id.button4);
		quitButton =(Button)findViewById(R.id.button5);
		restartButton=(Button)findViewById(R.id.button6);
		stateButton=(Button)findViewById(R.id.button7);
		
		upButton.setOnClickListener(this);
		leftButton.setOnClickListener(this);
		downButton.setOnClickListener(this);
		rightButton.setOnClickListener(this);
		quitButton.setOnClickListener(this);
		restartButton.setOnClickListener(this);
		stateButton.setOnClickListener(this);
		
		
		motor1=(TextView)findViewById(R.id.textView2);
		motor2=(TextView)findViewById(R.id.textView4);
		motor3=(TextView)findViewById(R.id.textView6);
		motor4=(TextView)findViewById(R.id.textView8);
		roll=(TextView)findViewById(R.id.textView10);
		pitch=(TextView)findViewById(R.id.textView12);
		yaw=(TextView)findViewById(R.id.textView14);
		ipInfo=(TextView)findViewById(R.id.textView16);
		
		motor1.setText(Datas.motor1Speed);
		motor2.setText(Datas.motor2Speed);
		motor3.setText(Datas.motor3Speed);
		motor4.setText(Datas.motor4Speed);
		
		roll.setText(Datas.rollValue);
		pitch.setText(Datas.pitchValue);
		yaw.setText(Datas.yawValue);
		
		ipInfo.setText(Datas.clientIP);
		
		stateGetClass=new StateGetClass(this);
		stateGetClass.start();
	}

	@Override
	public void recreate() {
		// TODO Auto-generated method stub
		super.recreate();
		Log.i("controlActivity","recreate");
		
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
		Log.i("controlActivity","resume");
		stateGetClass.setNotify();
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub

		switch (flag) {
		case 0: // receive
			byte[] receiveData = new byte[1024];
			try {
				Log.i("recv", "recv");
				dataInputStream=new DataInputStream(SocketClass.socket.getInputStream());
				dataInputStream.read(receiveData);

				receiveDataToString = new String(receiveData).split("\n")[0];

			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		case 1: // send
			try {
				//Log.i("send", new String(sendBuffer));
				dataOutputStream=new DataOutputStream(SocketClass.socket.getOutputStream());
				dataOutputStream.write(sendBuffer);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		}
	}
	
	private void send(byte[] data)
	{
		flag=1;
		sendBuffer=data;
	}
	
	private void recv()
	{
		flag=0;
	}
	
	
	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		
		String dataForSendString;
		byte[] dataForSend = new String("\n").getBytes();
		switch(v.getId())
		{
		case R.id.button1:  //up
			dataForSendString="up\n";
			dataForSend=dataForSendString.getBytes();
			break;
		case R.id.button2:   //left
			break;
		case R.id.button3:  //down
			dataForSendString="down\n";
			dataForSend=dataForSendString.getBytes();			
			break;
		case R.id.button4:
			break;
		case R.id.button5:
			dataForSendString="quit\n";
			dataForSend=dataForSendString.getBytes();
			break;
		case R.id.button6:
			dataForSendString="restart\n";
			dataForSend=dataForSendString.getBytes();
			break;
		case R.id.button7:
			dataForSendString="state\n";
			dataForSend=dataForSendString.getBytes();
			break;
		}
		
		stateGetClass.setWait();
		while(true)
		{
			if(stateGetClass.getWaitCheckFlag())
			{
				break;
			}
		}
		///////////////////////////////////////////////////////////////////////////////////
		thread=new Thread(this);
		send(dataForSend);
		thread.start();
		try {
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		thread=new Thread(this);
		recv();
		thread.start();
		try {
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		///////////////////////////////////////////////////////////////////////////////////
		stateGetClass.setNotify();
		
		setMotorSensorValue();
		
		
		try {
			Thread.sleep(Datas.serverSamplingTime);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		String [] recvData=receiveDataToString.split(",");
		
		Datas.motor1Speed=recvData[0];
		Datas.motor2Speed=recvData[1];
		Datas.motor3Speed=recvData[2];
		Datas.motor4Speed=recvData[3];
		Datas.rollValue=recvData[4];
		Datas.pitchValue=recvData[5];
		Datas.yawValue=recvData[6];
		
		setMotorSensorValue();
	}
	
	
	public void setMotorSensorValue()
	{
		motor1.setText(Datas.motor1Speed);
		motor2.setText(Datas.motor2Speed);
		motor3.setText(Datas.motor3Speed);
		motor4.setText(Datas.motor4Speed);
		
		roll.setText(Datas.rollValue);
		pitch.setText(Datas.pitchValue);
		yaw.setText(Datas.yawValue);
	}
	
	

}
