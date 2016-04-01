package com.example.controller3;

import android.app.ActionBar.LayoutParams;
import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

public class VerticalSeekBarLayout extends LinearLayout implements OnSeekBarChangeListener{

	private TextView seekBarValueTextView;
	private VerticalSeekBar verticalSeekBar;
	private SocketClass socketClass;
	private byte[] bByte;
	
	public VerticalSeekBarLayout(Context context,int width,int height) {
		super(context);
		// TODO Auto-generated constructor stub
		
		setOrientation(VERTICAL);
		LayoutParams param=new LayoutParams(width,height);
		param.setMargins(0, 0, 0, 100);
		setLayoutParams(param);
		
		seekBarValueTextView=new TextView(context);
		//seekBarValueTextView.setTextSize(TypedValue.COMPLEX_UNIT_SP,10);
		seekBarValueTextView.setGravity(Gravity.CENTER);
		seekBarValueTextView.setText("126");
		addView(seekBarValueTextView);
		
		verticalSeekBar=new VerticalSeekBar(context);
		
		verticalSeekBar.setMax(124);
		verticalSeekBar.setOnSeekBarChangeListener(this);
		addView(verticalSeekBar,new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.MATCH_PARENT));
		setGravity(Gravity.CENTER);
		bByte=new byte[1];
	}
	
	public void setProgress(int value)
	{
		verticalSeekBar.setProgress(value);
		seekBarValueTextView.setText(String.valueOf(value+126));
	}

	@Override
	public void onProgressChanged(SeekBar seekBar, int progress,
			boolean fromUser) {
		// TODO Auto-generated method stub
		seekBarValueTextView.setText(String.valueOf(progress+126));
		socketClass=SocketClass.getInstance();
		if(socketClass!=null)
		{
			//bByte[0]=3;
			//socketClass.write(bByte);
			bByte[0]=(byte) (progress+126);
			socketClass.write(bByte);
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		//Log.i("seekbar", String.valueOf(progress+126));
	}

	@Override
	public void onStartTrackingTouch(SeekBar seekBar) {
		// TODO Auto-generated method stub
		//seekBarValueTextView.setText(seekBar.getProgress());
		//Log.i("seekbar", String.valueOf(seekBar.getProgress()));
	}

	@Override
	public void onStopTrackingTouch(SeekBar seekBar) {
		// TODO Auto-generated method stub
		//seekBarValueTextView.setText(seekBar.getProgress());
		//Log.i("seekbar", String.valueOf(seekBar.getProgress()));
	}
	
	
	
	

}
