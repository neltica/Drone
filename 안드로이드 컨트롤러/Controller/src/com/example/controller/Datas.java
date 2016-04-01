package com.example.controller;

import java.net.Socket;

public class Datas {
	
	public static Socket socket;
	public byte[] recvbuffer;
	public byte[] sendBuffer;
	
	public float m1,m2,m3,m4;
	public Datas()
	{
		m1=0;
		m2=0;
		m3=0;
		m4=0;
	}

	public void splitMotorSpeed()
	{
		String s=new String(recvbuffer);
		if(s.indexOf(0)!=0)
		{
			s.subSequence(0, s.indexOf(0));
			String[] str = s.split(",");
			m1 = Float.parseFloat(str[0]);
			m2 = Float.parseFloat(str[1]);
			m3 = Float.parseFloat(str[2]);
			m4 = Float.parseFloat(str[3]);
		}
	}
}
