package planning_modules;

import java.util.LinkedList;
import java.util.Queue;

import commands.HighLevelCommand;
import commands.MidLevelCommand;

/**
 * This class creates a queue containing the mid-level commands required to 
 * carry out a high level command
 * @author Shota
 *
 */
public class Deposition {

	/**
	 * This is just a placeholder until I know what the actual function to check
	 * the position is
	 */
	public static final float binPosition = 0;

	/**
	 * This is just a placeholder until I know what the actual function to check
	 * the amount in the bin is
	 */
	public static final float load = 60;

	public static Queue<MidLevelCommand> receiveCommand(HighLevelCommand highCommand) {
		if (highCommand.getType() == 2) {
			int duration = 0;
			float amount = 0;
			
			//I do realize that there could be some errors with this system, but I'll get to that later
			if (highCommand.getIdentifier().equals(HighLevelCommand.amount_identifier)){
				amount = (float) highCommand.getData(0);
			}
			if (highCommand.getIdentifier().equals(HighLevelCommand.duration_identifier)){
				duration = (int) highCommand.getData(0);
			}
				
			return assignCommands(duration, amount);
		}
		else{
			System.err.println("Wrong type of Command");
			return null;
		}
		//return assignCommands(command.getIntDuration(), command.getFracDuration(), command.getAmount());
	}

	/**
	 * Checks the position of the bin
	 * 
	 * @return The bin's position as a float between 0 and 1
	 */
	public static float checkPosition() {
		// Insert some fuction to check the bin's position
		return binPosition;
	}

	/**
	 * Checks the amount currently in the bin
	 * 
	 * @return The load in the bin as a float
	 */
	public static float checkLoad() {
		return load;
	}

	public static Queue<MidLevelCommand> assignCommands(int duration, float amount) {
		Queue<MidLevelCommand> output = new LinkedList<MidLevelCommand>();

		// If there is stuff in the bin
		if (checkLoad() > 0) {
			// if the bin isn’t fully extended
			if (checkPosition() < 1)
				output.add(extend());

			output.add(conveyorOn());
			
			//If there is a time, there should be no amount specified
			if (duration > 0)
				output.add(waitTime(duration));
			//If there is no time, should be no amount specified.
			//Might be an error here if amount == -1
			else
				output.add(waitWeight(amount));
			
			output.add(conveyorOff());
			output.add(retract());
		}
		return output;
	}

	// Returns a mid-level command for extending all the way
	public static MidLevelCommand extend() {
		// return something cool
		return new MidLevelCommand("extend bin");
	}

	// Returns a mid-level command for retracting all da way
	public static MidLevelCommand retract() {
		// Return something awesome
		return new MidLevelCommand("retract bin");
	}

	// Returns a command for turning off the conveyor belt
	public static MidLevelCommand conveyorOn() {
		// Return something fantastic
		return new MidLevelCommand("Conveyor On");
	}

	// I think you get the point
	public static MidLevelCommand conveyorOff() {
		// Yes
		return new MidLevelCommand("Conveyor Off");
	}

	// This method is for deploying a command for waiting a certain amount of
	// time in ms (unit tbd)
	public static MidLevelCommand waitTime(int duration) {
		// I actually don’t know if we can add a command for waiting :)
		return new MidLevelCommand("Wait " + duration + " seconds");
	}

	// Same deal, but until certain value of load cell is achieved
	public static MidLevelCommand waitWeight(float weight) {
		// Same deal as above
		return new MidLevelCommand("Wait until " + weight + " kg is reached");
	}

}
