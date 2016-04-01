package com.example.controller;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;

import android.app.Activity;
import android.content.DialogInterface;
import android.content.DialogInterface.OnClickListener;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewDebug.FlagToString;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends Activity implements android.view.View.OnClickListener,Runnable{

	private Button p1UpButton, p2UpButton, p3UpButton, p4UpButton;
	private Button i1UpButton, i2UpButton, i3UpButton, i4UpButton;
	private Button d1UpButton, d2UpButton, d3UpButton, d4UpButton;

	private Button p1DownButton, p2DownButton, p3DownButton, p4DownButton;
	private Button i1DownButton, i2DownButton, i3DownButton, i4DownButton;
	private Button d1DownButton, d2DownButton, d3DownButton, d4DownButton;

	private TextView p1TextView, p2TextView, p3TextView, p4TextView;
	private TextView i1TextView, i2TextView, i3TextView, i4TextView;
	private TextView d1TextView, d2TextView, d3TextView, d4TextView;
	
	private TextView motor1TextView, motor2TextView, motor3TextView, motor4TextView;
	
	private TextView averageTextView;
	
	private Button motorDown,motorUp;
	
	private EditText ipEditText;
	
	
	private int p1Number,p2Number,p3Number,p4Number;
	private int i1Number,i2Number,i3Number,i4Number;
	private int d1Number,d2Number,d3Number,d4Number;
	
	
	private int motor1Number,motor2Number,motor3Number,motor4Number;
	private int averageMotor;
	
	private Thread thread;
	
	
	private Datas datas;
	private AcceptThread accept;
	private SendThread sendThread;
	private RecvThread recvThread;
	private Socket socket;
	private String ip;
	private int port;
	private InputStream inputStream;
	private OutputStream outputStream;
	private DataInputStream dataInputStream;
	private DataOutputStream dataOutputStream;
	
	private int bufferSize=1024;
	
	private String StringData;
	private byte[] byteData;
	private String[] message = null;
	
	private boolean startFlag,endFlag;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		
		datas=new Datas();
		bufferSize=1024;
		ip=null;
		port=6000;
		startFlag=false;
		endFlag=false;
		
		p1TextView=(TextView)findViewById(R.id.textView13);
		p2TextView=(TextView)findViewById(R.id.textView16);
		p3TextView=(TextView)findViewById(R.id.textView19);
		p4TextView=(TextView)findViewById(R.id.textView22);
		i1TextView=(TextView)findViewById(R.id.textView14);
		i2TextView=(TextView)findViewById(R.id.textView17);
		i3TextView=(TextView)findViewById(R.id.textView20);
		i4TextView=(TextView)findViewById(R.id.textView23);
		d1TextView=(TextView)findViewById(R.id.textView15);
		d2TextView=(TextView)findViewById(R.id.textView18);
		d3TextView=(TextView)findViewById(R.id.textView21);
		d4TextView=(TextView)findViewById(R.id.textView24);
		motor1TextView=(TextView)findViewById(R.id.TextView01);
		motor2TextView=(TextView)findViewById(R.id.TextView03);
		motor3TextView=(TextView)findViewById(R.id.TextView06);
		motor4TextView=(TextView)findViewById(R.id.TextView07);
		averageTextView=(TextView)findViewById(R.id.textView26);
		
		p1Number=0;
		p2Number=0;
		p3Number=0;
		p4Number=0;
		i1Number=0;
		i2Number=0;
		i3Number=0;
		i4Number=0;
		d1Number=0;
		d2Number=0;
		d3Number=0;
		d4Number=0;
		
		motor1Number=0;
		motor2Number=0;
		motor3Number=0;
		motor4Number=0;
		averageMotor=0;
		
		p1TextView.setText(String.valueOf(p1Number));
		p2TextView.setText(String.valueOf(p2Number));
		p3TextView.setText(String.valueOf(p3Number));
		p4TextView.setText(String.valueOf(p4Number));
		i1TextView.setText(String.valueOf(i1Number));
		i2TextView.setText(String.valueOf(i2Number));
		i3TextView.setText(String.valueOf(i3Number));
		i4TextView.setText(String.valueOf(i4Number));
		d1TextView.setText(String.valueOf(d1Number));
		d2TextView.setText(String.valueOf(d2Number));
		d3TextView.setText(String.valueOf(d3Number));
		d4TextView.setText(String.valueOf(d4Number));
		motor1TextView.setText(String.valueOf(motor1Number));
		motor2TextView.setText(String.valueOf(motor2Number));
		motor3TextView.setText(String.valueOf(motor3Number));
		motor4TextView.setText(String.valueOf(motor4Number));
		averageTextView.setText(String.valueOf(averageMotor));
		
		
		p1UpButton=(Button)findViewById(R.id.Button01);
		p2UpButton=(Button)findViewById(R.id.Button04);
		p3UpButton=(Button)findViewById(R.id.Button07);
		p4UpButton=(Button)findViewById(R.id.Button11);
		
		p1DownButton=(Button)findViewById(R.id.button1);
		p2DownButton=(Button)findViewById(R.id.Button4);
		p3DownButton=(Button)findViewById(R.id.Button7);
		p4DownButton=(Button)findViewById(R.id.Button10);
		
		i1UpButton=(Button)findViewById(R.id.Button02);
		i2UpButton=(Button)findViewById(R.id.Button05);
		i3UpButton=(Button)findViewById(R.id.Button08);
		i4UpButton=(Button)findViewById(R.id.Button13);
		
		i1DownButton=(Button)findViewById(R.id.Button2);
		i2DownButton=(Button)findViewById(R.id.Button5);
		i3DownButton=(Button)findViewById(R.id.Button8);
		i4DownButton=(Button)findViewById(R.id.Button12);
		
		d1UpButton=(Button)findViewById(R.id.Button03);
		d2UpButton=(Button)findViewById(R.id.Button06);
		d3UpButton=(Button)findViewById(R.id.Button09);
		d4UpButton=(Button)findViewById(R.id.Button15);
		
		d1DownButton=(Button)findViewById(R.id.Button3);
		d2DownButton=(Button)findViewById(R.id.Button6);
		d3DownButton=(Button)findViewById(R.id.Button9);
		d4DownButton=(Button)findViewById(R.id.Button14);
		
		motorDown=(Button)findViewById(R.id.button16);
		motorUp=(Button)findViewById(R.id.button17);
		
		ipEditText=(EditText)findViewById(R.id.editText1);
		
		
		p1UpButton.setOnClickListener(this);
		p2UpButton.setOnClickListener(this);
		p3UpButton.setOnClickListener(this);
		p4UpButton.setOnClickListener(this);
		
		p1DownButton.setOnClickListener(this);
		p2DownButton.setOnClickListener(this);
		p3DownButton.setOnClickListener(this);
		p4DownButton.setOnClickListener(this);
		
		i1UpButton.setOnClickListener(this);
		i2UpButton.setOnClickListener(this);
		i3UpButton.setOnClickListener(this);
		i4UpButton.setOnClickListener(this);
		
		i1DownButton.setOnClickListener(this);
		i2DownButton.setOnClickListener(this);
		i3DownButton.setOnClickListener(this);
		i4DownButton.setOnClickListener(this);
		
		d1UpButton.setOnClickListener(this);
		d2UpButton.setOnClickListener(this);
		d3UpButton.setOnClickListener(this);
		d4UpButton.setOnClickListener(this);
		
		d1DownButton.setOnClickListener(this);
		d2DownButton.setOnClickListener(this);
		d3DownButton.setOnClickListener(this);
		d4DownButton.setOnClickListener(this);
		
		d4DownButton.setOnClickListener(this);
		
		motorDown.setOnClickListener(this);
		motorUp.setOnClickListener(this);
		
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		//getMenuInflater().inflate(R.menu.main, menu);
		
		menu.add(0, 0, Menu.NONE, "연결");
		menu.add(0, 1, Menu.NONE, "세팅");
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == 0) {
			ip=ipEditText.getText().toString();
			Log.i("ip", ip);
			if(ip!=null)
			{
//				accept=new AcceptThread(datas,ip, port);
//				accept.start();
//				
//				try {
//					accept.join();
//				} catch (InterruptedException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
				
				
//				sendThread=new SendThread(datas);
//				recvThread=new RecvThread(datas);
//				sendThread.start();
//				recvThread.start();
				
//				while(true)
//				{
//					if(sendThread.flag==true && recvThread.flag==true)
//					{
//						break;
//					}
//				}
				
				thread=new Thread(this);
				thread.start();
			}
			else
			{
				Log.i("ip", "ip입력해주세요.");
			}
			return true;
		}
		else if(id==1)
		{
			//세팅
		}
		return super.onOptionsItemSelected(item);
	}
	

	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		
		String str;
		int elseLength;
		byte[] temp;
		
		switch (v.getId()) {
		case R.id.button1:
			Log.i("buttonclick", "1번p low Down");
			p1Number--;
			p1TextView.setText(String.valueOf(p1Number));
			break;
		case R.id.Button01:
			Log.i("buttonclick", "1번p high Down");
			p1Number++;
			p1TextView.setText(String.valueOf(p1Number));
			break;
		case R.id.Button2:
			Log.i("buttonclick", "1번i low Down");
			i1Number--;
			i1TextView.setText(String.valueOf(i1Number));
			break;
		case R.id.Button02:
			Log.i("buttonclick", "1번i high Down");
			i1Number++;
			i1TextView.setText(String.valueOf(i1Number));
			break;
		case R.id.Button3:
			Log.i("buttonclick", "1번d low Down");
			d1Number--;
			d1TextView.setText(String.valueOf(d1Number));
			break;
		case R.id.Button03:
			Log.i("buttonclick", "1번d high Down");
			d1Number++;
			d1TextView.setText(String.valueOf(d1Number));
			break;
		case R.id.Button4:
			Log.i("buttonclick", "2번p low Down");
			p2Number--;
			p2TextView.setText(String.valueOf(p2Number));
			break;
		case R.id.Button04:
			Log.i("buttonclick", "2번p high Down");
			p2Number++;
			p2TextView.setText(String.valueOf(p2Number));
			break;
		case R.id.Button5:
			Log.i("buttonclick", "2번i low Down");
			i2Number--;
			i2TextView.setText(String.valueOf(i2Number));
			break;
		case R.id.Button05:
			Log.i("buttonclick", "2번i high Down");
			i2Number++;
			i2TextView.setText(String.valueOf(i2Number));
			break;
		case R.id.Button6:
			Log.i("buttonclick", "2번d low Down");
			d2Number--;
			d2TextView.setText(String.valueOf(d2Number));
			break;
		case R.id.Button06:
			Log.i("buttonclick", "2번d high Down");
			d2Number++;
			d2TextView.setText(String.valueOf(d2Number));
			break;
		case R.id.Button7:
			Log.i("buttonclick", "3번p low Down");
			p3Number--;
			p3TextView.setText(String.valueOf(p3Number));
			break;
		case R.id.Button07:
			Log.i("buttonclick", "3번p high Down");
			p3Number++;
			p3TextView.setText(String.valueOf(p3Number));
			break;
		case R.id.Button8:
			Log.i("buttonclick", "3번i low Down");
			i3Number--;
			i3TextView.setText(String.valueOf(i3Number));
			break;
		case R.id.Button08:
			Log.i("buttonclick", "3번i high Down");
			i3Number++;
			i3TextView.setText(String.valueOf(i3Number));
			break;
		case R.id.Button9:
			Log.i("buttonclick", "3번d low Down");
			d3Number--;
			d3TextView.setText(String.valueOf(d3Number));
			break;
		case R.id.Button09:
			Log.i("buttonclick", "3번d high Down");
			d3Number++;
			d3TextView.setText(String.valueOf(d3Number));
			break;
		case R.id.Button10:
			Log.i("buttonclick", "4번p low Down");
			p4Number--;
			p4TextView.setText(String.valueOf(p4Number));
			break;
		case R.id.Button11:
			Log.i("buttonclick", "4번p high Down");
			p4Number++;
			p4TextView.setText(String.valueOf(p4Number));
			break;
		case R.id.Button12:
			Log.i("buttonclick", "4번i low Down");
			i4Number--;
			i4TextView.setText(String.valueOf(i4Number));
			break;
		case R.id.Button13:
			Log.i("buttonclick", "4번i high Down");
			i4Number++;
			i4TextView.setText(String.valueOf(i4Number));
			break;
		case R.id.Button14:
			Log.i("buttonclick", "4번d low Down");
			d4Number--;
			d4TextView.setText(String.valueOf(d4Number));
			break;
		case R.id.Button15:
			Log.i("buttonclick", "4번d high Down");
			d4Number++;
			d4TextView.setText(String.valueOf(d4Number));
			break;
		case R.id.button16:
			Log.i("buttonclick", "motor low Down");
			averageMotor--;
			averageTextView.setText(String.valueOf(averageMotor));
			str="md,-1\n";
			//str=String.format("%04.4f", averageMotor)+" motor low down\n";
			
			
			startFlag=true;
			byteData=new byte[bufferSize];
			temp=str.getBytes();
			System.arraycopy(temp, 0, byteData,0 , temp.length);
			endFlag=true;
//			sendThread.send(byteData);
			break;
		case R.id.button17:
			Log.i("buttonclick", "motor high Down");
			averageMotor++;
			averageTextView.setText(String.valueOf(averageMotor));
			str="mu,1\n";
			//str=String.format("%04.4f", averageMotor)+" motor high down\n";
			
			startFlag=true;
			byteData=new byte[bufferSize];
			temp=str.getBytes();
			System.arraycopy(temp, 0, byteData,0 , temp.length);
			endFlag=true;
//			sendThread.send(byteData);
			break;

		}
	}

//	@Override
//	public void run() {
//		// TODO Auto-generated method stub
//		
//		String str;
//		str="getMotorSpeed\n";		
//		
//		while(true)
//		{
//			byteData=str.getBytes();
//			sendThread.send(byteData);
//			datas.splitMotorSpeed();
//			mHandler.sendEmptyMessage(0); // sendMessage(Message msg)도 사용가능
//			try {
//				Thread.sleep(10);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//		}
//		
//	}
	
	
	
	Handler mHandler = new Handler() {
	     public void handleMessage(Message msg) {
	      if(msg.what == 0) {
	    	motor1TextView.setText(message[1]);
			motor2TextView.setText(message[2]);
			motor3TextView.setText(message[3]);
			motor4TextView.setText(message[4]);
	      }
	  }
	};

	@Override
	public void run() {
		// TODO Auto-generated method stub

		String str;
		str = "getMotorSpeed\n";
		byte[] temp=str.getBytes();

		
		try {
			datas.socket = new Socket(ip, port);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		try {
			inputStream = datas.socket.getInputStream();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			outputStream = datas.socket.getOutputStream();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		dataInputStream = new DataInputStream(inputStream);
		dataOutputStream = new DataOutputStream(outputStream);

		while (true) {
			if(startFlag)
			{
				startFlag=false;
				while(true)
				{
					if(endFlag)
					{
						endFlag=false;
						break;
					}
				}
			}
			else
			{
				byteData=new byte[bufferSize];
				System.arraycopy(temp, 0, byteData, 0, temp.length);
			}
			try {
				dataOutputStream.write(byteData);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			byteData = new byte[bufferSize];
			try {
				dataInputStream.read(byteData);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			///////////////////
			String s=new String(byteData);
			int index=s.indexOf("\n");
			if(index!=-1)
			{
				s=s.substring(0, index);
				message=s.split(",");
				if(message[0].equals("ms"))
				{
					mHandler.sendEmptyMessage(0); // sendMessage(Message msg)도 사용가능
				}
			}
			
			
			///////////////////
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
	
	
	
	
	
	

	

	

	
}
