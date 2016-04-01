package com.example.controller;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;

public class SendThread extends Thread{

	private Datas datas;
	private Socket socket;
	private OutputStream outputStream;
	private DataOutputStream dataOutputStream;
	private byte[] buffer;
	public boolean flag;
	
	
	public SendThread(Datas datas) {
		// TODO Auto-generated constructor stub
		this.datas=datas;
		this.socket=datas.socket;
		flag=false;
	}
	
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		super.run();
		
		try {
			outputStream=socket.getOutputStream();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		dataOutputStream=new DataOutputStream(outputStream);
		
		
		while(true)
		{
			flag=true;
			waitting();
			
			
			try {
				dataOutputStream.write(buffer);  //°íÄ¥ºÎºÐ
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
		
		
	}
	
	public synchronized void send(byte[] buffer)
	{
		this.buffer=buffer;
		notify();
	}
	public synchronized void waitting()
	{
		try {
			wait();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
