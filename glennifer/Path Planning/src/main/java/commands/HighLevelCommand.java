package commands;

import java.util.Calendar;
import java.util.List;

/**
 * This are the current data values needing to be set in the high level command.
 * @author Shota
 *
 */
public class HighLevelCommand {
	
	public static final String amount_identifier = "AMOUNT";
	public static final String duration_identifier = "DURATION";

	/**
	 * This field serves as the time stamp of when this command was issued
	 */
	long timeStamp;
	
	/**
	 * This field serves as the Identification number of the command,
	 * and should be a nonnegative integer.
	 */
	int id;

	/**
	 * This field serves as the indicator for what type of operation the command specifies.
	 * 1: Locomotion
	 * 2: Deposition
	 * 3: Excavation
	 */
	int type;
	
	/**
	 * This field helps distinguish certain commands between specifying an amount and a duration
	 */
	String identifier;

	/**
	 * This array holds the specific details of the command
	 */
	double[] data;
	
	/**
	 * Creates a High level command for Locomotion
	 * @param id 
	 * @param x X coordinate of the destination
	 * @param y Y coordinate of the destination
	 * @param z Z coordinate of the destination
	 */
	public HighLevelCommand(int id, float x, float y, float z){
		this.type = 1;
		this.id = id;
		this.timeStamp = Calendar.getInstance().getTimeInMillis();
		this.data = new double[]{x,y,z};
	}
	
	/**
	 * Creates a High level command for Deposition with a specified amount remaining
	 * @param id
	 * @param amount
	 */
	public HighLevelCommand(int id, float amount){
		this.type = 2;
		this.id = id;
		this.identifier = this.amount_identifier;
		this.timeStamp = Calendar.getInstance().getTimeInMillis();
		this.data = new double[]{amount};
	}
	
	/**
	 * Creates a High level command for Deposition with a specified duration
	 * @param id
	 * @param duration The duration of the dig in seconds
	 */
	public HighLevelCommand(int id, int duration){
		this.type = 2;
		this.id = id;
		this.identifier = this.duration_identifier;
		this.timeStamp = Calendar.getInstance().getTimeInMillis();
		this.data = new double[]{duration};
	}
	
	/**
	 * Creates a High level command for Excavation with a specified amount in the bin remaining
	 * @param id
	 * @param depth
	 * @param amount
	 */
	public HighLevelCommand(int id, float depth, float amount){
		this.type = 3;
		this.id = id;
		this.identifier = this.amount_identifier;
		this.timeStamp = Calendar.getInstance().getTimeInMillis();
		this.data = new double[]{depth, amount};
	}
	
	/**
	 * Creates a High level command for Excavation with a specified amount in the bin remaining
	 * @param id
	 * @param depth
	 * @param duration of the excavation in seconds
	 */
	public HighLevelCommand(int id, float depth, int duration){
		this.type = 3; 
		this.id = id;
		this.identifier = this.duration_identifier;
		this.timeStamp = Calendar.getInstance().getTimeInMillis();
		this.data = new double[]{depth, duration};
	}
	public long getTimeStamp(){
		return this.timeStamp;
	}
	
	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
	
	public int getType() {
		return type;
	}

	public double[] getData() {
		return data;
	}
	
	public double getData(int index) {
		return data[index];
	}
	
	public String getIdentifier(){
		return identifier;
	}
}
