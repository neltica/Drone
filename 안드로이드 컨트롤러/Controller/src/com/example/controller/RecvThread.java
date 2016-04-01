package com.example.controller;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;

public class RecvThread extends Thread {
	
	private Datas datas;
	private Socket socket;
	private InputStream inputStream;
	private DataInputStream dataInputStream;
	public boolean flag;
	
	
	public RecvThread(Datas datas)
	{
		this.datas=datas;
		this.socket=datas.socket;
		flag=false;
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		super.run();
		
		try {
			inputStream=socket.getInputStream();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		dataInputStream=new DataInputStream(inputStream);
		
		
		while(true)
		{
			flag=true;
			try {
				
				datas.recvbuffer=new byte[1024];
				synchronized (datas.recvbuffer) {
					dataInputStream.read(datas.recvbuffer);
				}
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		
		
	}
	
	

}
