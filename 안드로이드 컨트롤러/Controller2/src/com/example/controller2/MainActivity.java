package com.example.controller2;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

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

public class MainActivity extends Activity implements OnClickListener,Runnable {

	
	private Button button;
	private EditText ipEditText;

	
	
	@Override
	protected void onDestroy() {
		// TODO Auto-generated method stub
		super.onDestroy();
		Log.i("onDestroy","onDestroy");
		try {
			SocketClass.socket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		button=(Button)findViewById(R.id.button1);
		ipEditText=(EditText)findViewById(R.id.ipEditText);
		
		button.setOnClickListener(this);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == R.id.action_settings) {
			return true;
		}
		return super.onOptionsItemSelected(item);
	}

	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub

		switch (v.getId()) {
		case R.id.button1:
			Intent intent = new Intent(getApplicationContext(),ControlActivity.class);
			Thread thread = new Thread(this);
			thread.start();
			try {
				thread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			startActivity(intent);
			break;
		}

	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		
			try {
				String ipString=ipEditText.getText().toString();
				SocketClass.socket = new Socket(ipString, 6000);
				
				DataInputStream firstDataInputStream=new DataInputStream(SocketClass.socket.getInputStream());
				
				byte[] firstClientData= new byte[1024];
				
				firstDataInputStream.read(firstClientData);
				
				Log.i("clientInfo", new String(firstClientData));
				String [] clientDataStr=new String(firstClientData).split("\n")[0].split(",");
				Datas.motor1Speed=clientDataStr[0];
				Datas.motor2Speed=clientDataStr[1];
				Datas.motor3Speed=clientDataStr[2];
				Datas.motor4Speed=clientDataStr[3];
				Datas.rollValue=clientDataStr[4];
				Datas.pitchValue=clientDataStr[5];
				Datas.yawValue=clientDataStr[6];
				Datas.clientIP=SocketClass.socket.getInetAddress().getHostAddress();
				//SocketClass.server=new ServerSocket(6000);
				//SocketClass.connectFlag=true;
			} catch (UnknownHostException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		
	}

	@Override
	protected void onStop() {
		// TODO Auto-generated method stub
		super.onStop();
		Log.i("onStop","onStop");
	}
}
