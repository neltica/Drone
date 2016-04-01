package com.example.controller2;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.provider.ContactsContract.Contacts.Data;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;

public class PidSetActivity extends Activity implements Runnable,OnClickListener {

	private EditText p1Edit,p2Edit,p3Edit,p4Edit;
	private EditText i1Edit,i2Edit,i3Edit,i4Edit;
	private EditText d1Edit,d2Edit,d3Edit,d4Edit;
	private Button pidSettingButton;
	private int flag;
	
	private byte[] buffer;
	
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_pidset);
		
		
		Thread thread=new Thread(this);
		flag=1;
		sendData(new String("pidget").getBytes());
		thread.start();
		try {
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		thread=new Thread(this);
		flag=0;
		thread.start();
		try {
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		p1Edit=(EditText)findViewById(R.id.editText2);
		p2Edit=(EditText)findViewById(R.id.editText5);
		p3Edit=(EditText)findViewById(R.id.editText8);
		p4Edit=(EditText)findViewById(R.id.editText11);
		
		i1Edit=(EditText)findViewById(R.id.editText3);
		i2Edit=(EditText)findViewById(R.id.editText6);
		i3Edit=(EditText)findViewById(R.id.editText9);
		i4Edit=(EditText)findViewById(R.id.editText12);
		
		d1Edit=(EditText)findViewById(R.id.editText4);
		d2Edit=(EditText)findViewById(R.id.editText7);
		d3Edit=(EditText)findViewById(R.id.editText10);
		d4Edit=(EditText)findViewById(R.id.editText13);
		
		pidSettingButton=(Button)findViewById(R.id.button1);
		
		p1Edit.setText(String.valueOf(Datas.p1Gain));
		p2Edit.setText(String.valueOf(Datas.p2Gain));
		p3Edit.setText(String.valueOf(Datas.p3Gain));
		p4Edit.setText(String.valueOf(Datas.p4Gain));
		
		i1Edit.setText(String.valueOf(Datas.i1Gain));
		i2Edit.setText(String.valueOf(Datas.i2Gain));
		i3Edit.setText(String.valueOf(Datas.i3Gain));
		i4Edit.setText(String.valueOf(Datas.i4Gain));
		
		d1Edit.setText(String.valueOf(Datas.d1Gain));
		d2Edit.setText(String.valueOf(Datas.d2Gain));
		d3Edit.setText(String.valueOf(Datas.d3Gain));
		d4Edit.setText(String.valueOf(Datas.d4Gain));
		
		pidSettingButton.setOnClickListener(this);
		
	}


	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		DataInputStream dataInputStream=null;
		DataOutputStream dataOutputStream=null;
		try {
			dataInputStream= new DataInputStream(SocketClass.socket.getInputStream());
			dataOutputStream= new DataOutputStream(SocketClass.socket.getOutputStream());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		switch(flag)
		{
		case 0:
			try {
				byte[] receiveData=new byte[1024];
				dataInputStream.read(receiveData);
				String receiveDataToString= new String(receiveData).split("\n")[0];
				
				String [] splitPidData=receiveDataToString.split(",");
				
				Datas.p1Gain=Float.parseFloat(splitPidData[0]);
				Datas.i1Gain=Float.parseFloat(splitPidData[1]);
				Datas.d1Gain=Float.parseFloat(splitPidData[2]);
				Datas.p2Gain=Float.parseFloat(splitPidData[3]);
				Datas.i2Gain=Float.parseFloat(splitPidData[4]);
				Datas.d2Gain=Float.parseFloat(splitPidData[5]);
				Datas.p3Gain=Float.parseFloat(splitPidData[6]);
				Datas.i3Gain=Float.parseFloat(splitPidData[7]);
				Datas.d3Gain=Float.parseFloat(splitPidData[8]);
				Datas.p4Gain=Float.parseFloat(splitPidData[9]);
				Datas.i4Gain=Float.parseFloat(splitPidData[10]);
				Datas.d4Gain=Float.parseFloat(splitPidData[11]);			
				
				handler.sendEmptyMessage(0);
				
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}			
			break;
		case 1:
			try {
				dataOutputStream.write(buffer);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		}
	}

	private void sendData(byte[] buffer)
	{
		this.buffer=buffer;
	}

	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		switch(v.getId())
		{
		case R.id.button1:
			setPidSetting();
			Thread thread=new Thread(this);
			flag=1;
			sendData(new String("pidset,"+String.valueOf(Datas.p1Gain)+","+String.valueOf(Datas.i1Gain)+","+String.valueOf(Datas.d1Gain)+","+String.valueOf(Datas.p2Gain)+","+String.valueOf(Datas.i2Gain)+","+String.valueOf(Datas.d2Gain)+","+String.valueOf(Datas.p3Gain)+","+String.valueOf(Datas.i3Gain)+","+String.valueOf(Datas.d3Gain)+","+String.valueOf(Datas.p4Gain)+","+String.valueOf(Datas.i4Gain)+","+String.valueOf(Datas.d4Gain)+"\n").getBytes());
			thread.start();
			try {
				thread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			thread=new Thread(this);
			flag=0;
			thread.start();
			try {
				thread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			
			break;
		}
	}
	
	Handler handler = new Handler()
	{
		public void handleMessage(final Message msg)
		{
			p1Edit.setText(String.valueOf(Datas.p1Gain));
			p2Edit.setText(String.valueOf(Datas.p2Gain));
			p3Edit.setText(String.valueOf(Datas.p3Gain));
			p4Edit.setText(String.valueOf(Datas.p4Gain));
			
			i1Edit.setText(String.valueOf(Datas.i1Gain));
			i2Edit.setText(String.valueOf(Datas.i2Gain));
			i3Edit.setText(String.valueOf(Datas.i3Gain));
			i4Edit.setText(String.valueOf(Datas.i4Gain));
			
			d1Edit.setText(String.valueOf(Datas.d1Gain));
			d2Edit.setText(String.valueOf(Datas.d2Gain));
			d3Edit.setText(String.valueOf(Datas.d3Gain));
			d4Edit.setText(String.valueOf(Datas.d4Gain));
		}
	};
	
	private void setPidSetting()
	{
		Datas.p1Gain=Float.parseFloat(p1Edit.getText().toString());
		Datas.i1Gain=Float.parseFloat(i1Edit.getText().toString());
		Datas.d1Gain=Float.parseFloat(d1Edit.getText().toString());
		
		Datas.p2Gain=Float.parseFloat(p2Edit.getText().toString());
		Datas.i2Gain=Float.parseFloat(i2Edit.getText().toString());
		Datas.d2Gain=Float.parseFloat(d2Edit.getText().toString());
		
		Datas.p3Gain=Float.parseFloat(p3Edit.getText().toString());
		Datas.i3Gain=Float.parseFloat(i3Edit.getText().toString());
		Datas.d3Gain=Float.parseFloat(d3Edit.getText().toString());
		
		Datas.p4Gain=Float.parseFloat(p4Edit.getText().toString());
		Datas.i4Gain=Float.parseFloat(i4Edit.getText().toString());
		Datas.d4Gain=Float.parseFloat(d4Edit.getText().toString());
		
		
	}

}
