package com.example.controller3;



import android.app.ActionBar.LayoutParams;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.hardware.Camera.Parameters;
import android.os.Bundle;
import android.transition.SidePropagation;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.GridLayout;
import android.widget.LinearLayout;

public class MainActivity extends Activity implements OnTouchListener{

	private EditText ipEdit,portEdit;
	private Button connectButton,disconnectButton,startButton,stopButton,upButton,downButton;
	private SocketClass socketClass;
	private byte[] bByte;
	private int speed=0;
	
	private VerticalSeekBarLayout verticalSeekBarLayout;
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		LinearLayout linearLayout=(LinearLayout)findViewById(R.id.linearLayout2);
		linearLayout.setOrientation(LinearLayout.HORIZONTAL);		
		verticalSeekBarLayout = new VerticalSeekBarLayout(this,LayoutParams.MATCH_PARENT,LayoutParams.MATCH_PARENT);
		linearLayout.addView(verticalSeekBarLayout);
		
		ipEdit=(EditText)findViewById(R.id.editText1);
		portEdit=(EditText)findViewById(R.id.editText2);
		connectButton=(Button)findViewById(R.id.connect);
		connectButton.setOnTouchListener(this);
		
		disconnectButton=(Button)findViewById(R.id.disconnect);
		disconnectButton.setOnTouchListener(this);
		
		startButton=(Button)findViewById(R.id.start);
		startButton.setOnTouchListener(this);
		stopButton=(Button)findViewById(R.id.stop);
		stopButton.setOnTouchListener(this);
		
		upButton=(Button)findViewById(R.id.up);
		upButton.setOnTouchListener(this);
		downButton=(Button)findViewById(R.id.down);
		downButton.setOnTouchListener(this);
		
		bByte=new byte[1];
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
			Intent intent = new Intent(getApplicationContext(),PIDSettingActivity.class);
			startActivity(intent);
			return true;
		}
		return super.onOptionsItemSelected(item);
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {
		// TODO Auto-generated method stub
		
		
		if(event.getAction()==MotionEvent.ACTION_UP)
		{
			switch (v.getId()) {
			case R.id.connect:
				String ip = ipEdit.getText().toString();
				int port = Integer.parseInt(portEdit.getText().toString());
				socketClass = SocketClass.getInstance(ip, port);
				verticalSeekBarLayout.setProgress(0);
				break;

			case R.id.disconnect:
				socketClass = SocketClass.getInstance();
				if (socketClass != null) {
					socketClass.threadStop();
				}
				
				break;

			case R.id.start:
				socketClass = SocketClass.getInstance();
				if (socketClass != null) {
					bByte[0]=2;
					socketClass.write(bByte);
				}
				break;
			case R.id.stop:
				socketClass = SocketClass.getInstance();
				if (socketClass != null) {
					bByte[0]=1;
					socketClass.write(bByte);
				}
				break;
				
			case R.id.up:
				socketClass = SocketClass.getInstance();
				if (socketClass != null) {
					speed++;
					verticalSeekBarLayout.setProgress(speed);
					bByte[0]=4;
					socketClass.write(bByte);
				}
				break;
			case R.id.down:
				socketClass = SocketClass.getInstance();
				if (socketClass != null) {
					speed--;
					verticalSeekBarLayout.setProgress(speed);
					bByte[0]=5;
					socketClass.write(bByte);
				}
				break;
			}
		}
		return false;
	}

}
