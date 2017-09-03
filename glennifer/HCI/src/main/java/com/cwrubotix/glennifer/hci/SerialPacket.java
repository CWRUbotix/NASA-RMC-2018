package com.cwrubotix.glennifer.hci;

public class SerialPacket {
	public final byte command;
	public final byte[] data;
	SerialPacket(byte command, byte[] data) {
		this.command = command;
		this.data = data;
	}
	
	public byte[] asPacket() {
		byte[] out = new byte[2+data.length];
		out[0] = command;
		out[1] = (byte)(data.length);
		System.arraycopy(data, 0, out, 2, data.length);
		return out;
	}
}
