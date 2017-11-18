package main.java.com.cwrubotix.glennifer.automodule;

public class AutoTransit{
	private final Position DUMP_BIN = new Position(0.0F, 0.0F, Math.PI, 0.0F);
	/*Horizontal line representing where digging arena starts.*/
	private final Position DIGGING_AREA = new Position(0.0F, 4.41F, -1.0, 0.0F);
	private final float CLEARANCE_DIST = 0.3F; //Setting this to 30cm for now. Will have to change it after testing locomotion.
	private static Position currentPos;	
	
	/*
	 * TODO LIST
	 * 
	 * 1) Create Wrapper data type for coordinate values that we receive from localization
	 * 2) Subscribe to appropriate sensor values. (location within the arena, locomotion motors)
	 * 3) Come up with path planning algorithm.
	 * 4) Come up with possible errors and handling mechanism.
	 * 5) Set up the Connection Factory
	 * 
	 */
	
	public static Position getCurrentPos(){
		return currentPos;
	}
	
}
