package com.example.controller;

import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import android.util.Log;

public class AcceptThread extends Thread {

	private Datas datas;
	private String ip;
	private int port;
	public AcceptThread(Datas datas,String ip,int port) {
		// TODO Auto-generated constructor stub
		this.datas=datas;
		this.ip=ip;
		this.port=port;
	}
	@Override
	public void run() {
		// TODO Auto-generated method stub
		super.run();
		
		try {
			datas.socket=new Socket(ip,port);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Log.i("socket","connect");	
	}

}
