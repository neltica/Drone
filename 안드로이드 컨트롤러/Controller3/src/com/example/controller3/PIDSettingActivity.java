package com.example.controller3;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewGroup.LayoutParams;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Toast;

public class PIDSettingActivity extends Activity implements OnTouchListener {

	
	private SocketClass socketClass;
	private LinearLayout linearLayout,subLinearLayout;
	private ScrollView scrollView;
	private CustomEditText p1,i1,d1,p2,i2,d2,p3,i3,d3,p4,i4,d4;
	private Button setButton,getButton;
	private String bString;
	private byte[] buff;
	
	@Override
	protected void onStop() {
		// TODO Auto-generated method stub
		super.onStop();
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {
		// TODO Auto-generated method stub
		
		
		if(event.getAction()==MotionEvent.ACTION_UP)
		{
			Log.i("id",String.valueOf(v.getId())+","+String.valueOf(setButton.getId())+","+String.valueOf(getButton.getId()));
			switch(v.getId())
			{
			case 0:
				Log.i("setButton", "setButton");
				socketClass=SocketClass.getInstance();
				if(socketClass!=null)
				{
					bString = new String("pidset,");
					bString+=p1.getText()+",";
					bString+=i1.getText()+",";
					bString+=d1.getText()+",";
					bString+=p2.getText()+",";
					bString+=i2.getText()+",";
					bString+=d2.getText()+",";
					bString+=p3.getText()+",";
					bString+=i3.getText()+",";
					bString+=d3.getText()+",";
					bString+=p4.getText()+",";
					bString+=i4.getText()+",";
					bString+=d4.getText()+"\n";
					
					socketClass.write(bString.getBytes());
					buff=socketClass.recv();
					Toast.makeText(getApplicationContext(), new String(buff), Toast.LENGTH_SHORT).show();
				}
			
			break;
			case 1:
				Log.i("getButton", "getButton");
				socketClass=SocketClass.getInstance();
				if(socketClass!=null)
				{
					bString = new String("pidget\n");
					socketClass.write(bString.getBytes());
					buff=socketClass.recv();
					bString=new String(buff);
					String[] pidString=bString.split("\n")[0].split(",");
					Log.i("pidget", String.valueOf(pidString[0]));
					Log.i("pidget", String.valueOf(pidString[1]));
					Log.i("pidget", String.valueOf(pidString[2]));
					Log.i("pidget", String.valueOf(pidString[3]));
					Log.i("pidget", String.valueOf(pidString[4]));
					Log.i("pidget", String.valueOf(pidString[5]));
					Log.i("pidget", String.valueOf(pidString[6]));
					Log.i("pidget", String.valueOf(pidString[7]));
					Log.i("pidget", String.valueOf(pidString[8]));
					Log.i("pidget", String.valueOf(pidString[9]));
					Log.i("pidget", String.valueOf(pidString[10]));
					Log.i("pidget", String.valueOf(pidString[11]));
					
					p1.setText(pidString[0]);
					i1.setText(pidString[1]);
					d1.setText(pidString[2]);
					p2.setText(pidString[3]);
					i2.setText(pidString[4]);
					d2.setText(pidString[5]);
					p3.setText(pidString[6]);
					i3.setText(pidString[7]);
					d3.setText(pidString[8]);
					p4.setText(pidString[9]);
					i4.setText(pidString[10]);
					d4.setText(pidString[11]);
					
					
				}
				break;
			}
		}
		return false;
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		
		setContentView(R.layout.activity_pid);

		linearLayout=(LinearLayout)findViewById(R.id.LinearLayout1);
		setButton=new Button(this);
		setButton.setText("Set PID");
		setButton.setId(0);
		linearLayout.addView(setButton);
		getButton=new Button(this);
		getButton.setText("Get PID");
		getButton.setId(1);
		linearLayout.addView(getButton);
		
		setButton.setOnTouchListener(this);
		getButton.setOnTouchListener(this);
		
		scrollView=new ScrollView(this);
		linearLayout.addView(scrollView);
		
		subLinearLayout=new LinearLayout(this);
		subLinearLayout.setOrientation(LinearLayout.VERTICAL);
		
		scrollView.addView(subLinearLayout);
		
		p1=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"P1:");
		subLinearLayout.addView(p1);
		i1=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"I1:");
		subLinearLayout.addView(i1);
		d1=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"D1:");
		subLinearLayout.addView(d1);
		
		p2=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"P2:");
		subLinearLayout.addView(p2);
		i2=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"I2:");
		subLinearLayout.addView(i2);
		d2=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"D2:");
		subLinearLayout.addView(d2);
		
		p3=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"P3:");
		subLinearLayout.addView(p3);
		i3=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"I3:");
		subLinearLayout.addView(i3);
		d3=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"D3:");
		subLinearLayout.addView(d3);
		
		p4=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"P4:");
		subLinearLayout.addView(p4);
		i4=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"I4:");
		subLinearLayout.addView(i4);
		d4=new CustomEditText(this,LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT,6,"D4:");
		subLinearLayout.addView(d4);
		
		
	}

}
