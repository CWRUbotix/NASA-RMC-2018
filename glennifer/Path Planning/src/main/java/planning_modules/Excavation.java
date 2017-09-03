package planning_modules;

import java.util.LinkedList;
import java.util.Queue;

import commands.HighLevelCommand;
import commands.MidLevelCommand;

public class Excavation {

	/**
	 * I don't know the actual bucket conveyor length so I just took it as one
	 * meter
	 */
	public static final double assumedBeltLength = 1;

	/**
	 * I don't know the length of the arm holding up the conveyor belt, so I
	 * just guessed as 1/4 of a meter.
	 */
	public static final double assumedSupportArmLength = 0.25f;

	/**
	 * I don't know exactly how high above the ground the bottom point of the
	 * arm starts so I just assumed it to be half a meter (50 cm).
	 */
	public static final double assumedInitialClearance = .5f;

	/**
	 * see calculation sheet for this one
	 */
	public static final double assumedInitialAngle = Math.atan(assumedBeltLength / assumedSupportArmLength) + 90;

	/**
	 * The boolean that is true when excavation system is currently extended,
	 * false when retracted
	 */
	private static boolean extended = false;

	/**
	 * Receives the High Level Command and returns a queue of Mid Level Commands
	 * 
	 * @param highCommand:
	 *            a command for excavation that contains excavation height,
	 *            duration / amount
	 * @return Queue of Mid Level Commands to execute
	 */
	public static Queue<MidLevelCommand> receiveCommand(HighLevelCommand highCommand) {
		if (highCommand.getType() == 3) {
			float depth = 0;
			int duration = 0;
			float amount = 0;
			
			//I do realize that there could be some errors with this system, but I'll get to that later
			if (highCommand.getIdentifier().equals(HighLevelCommand.amount_identifier)){
				depth = (float) highCommand.getData(0);
				amount = (float) highCommand.getData(1);
			}
			if (highCommand.getIdentifier().equals(HighLevelCommand.duration_identifier)){
				depth = (float) highCommand.getData(0);
				duration = (int) highCommand.getData(1);
			}
				
			return assignCommands(depth, duration, amount);
		}
		else{
			System.err.println("Wrong type of Command");
			return null;
		}
	}

	/**
	 * The method that assigns the commands, requires information from a high
	 * level command
	 * 
	 * @param height
	 *            (negative height denotes under ground)
	 * @param intTime
	 * @param floatTime
	 * @param amount
	 * @return LinkedList of Mid Level Commands to execute. By default, the
	 *         robot's belt will be extended to the maximum length in order to
	 *         achieve the desired excavation height.
	 */
	public static Queue<MidLevelCommand> assignCommands(double height, int duration, float amount) {
		Queue<MidLevelCommand> output = new LinkedList<MidLevelCommand>();

		output.add(setBeltExtension(1));
		output.add(setArmAngle(calcAngle(height)));
		output.add(waitTime(duration));
		output.add(waitWeight(amount));
		return resetSystem(output);
	}

	/**
	 * 0 is defined as the ground, < 0 is below ground, > 0 is above ground.
	 * 
	 * @return The current height of the bottom point of the excavation system.
	 */
	public static double currentClearance() {
		// return assumedInitialClearance -
		// assumedSupportArmLength*(1-Math.cos(currentArmAngle())) -
		// currentBeltExtension()*Math.sin(currentArmAngle());
		return resultantLength() * Math.sin(currentArmAngle());
	}

	/**
	 * Returns how far the belt is currently extended. I currently do not know
	 * how to obtain them, so it will remain at max, wihich is arbitrarily set
	 * to 1.
	 * 
	 * @return Length of the extended excavation arm / belth
	 */
	public static double currentBeltExtension() {
		return assumedBeltLength;
	}

	/**
	 * Returns the angle at which the excavation arm / belt is tited at. I
	 * currently do not know how to obtain this value, so it will be set to a
	 * random value.
	 * 
	 * @return Angle of the arm / belt
	 */
	public static double currentArmAngle() {
		return assumedInitialAngle;
	}

	public static MidLevelCommand setBeltExtension(double length) {
		return new MidLevelCommand("Set belt extension to " + length + " meters");
	}

	/**
	 * Issues a command to set the arm angle at the given angle.
	 * 
	 * @return The command
	 */
	public static MidLevelCommand setArmAngle(double angle) {
		return new MidLevelCommand("Set arm angle to " + angle + " radians");
	}

	/**
	 * Issues a command to wait until the weight is achieved
	 * 
	 * @param weight
	 * @return the command
	 */
	public static MidLevelCommand waitWeight(float weight) {
		return new MidLevelCommand("Wait until " + weight + " kg is achieved");
	}

	/**
	 * Issues a command to wait for a certain duration
	 * 
	 * @param intTime
	 * @param fracTime
	 * @return The command
	 */
	public static MidLevelCommand waitTime(int duration) {
		// I actually don’t know if we can add a command for waiting :)
		return new MidLevelCommand("Wait " + duration + " seconds");
	}

	/**
	 * Adds the commands to return the excavation system to initial state at end
	 * of queue
	 */
	public static Queue<MidLevelCommand> resetSystem(Queue<MidLevelCommand> queue) {
		queue.add(setBeltExtension(0));
		queue.add(setArmAngle(0));

		return queue;
	}

	/**
	 * Calculates the arm angle required to achieve desired height. Currently
	 * assumes the arm will be fully extended when calculating angle.
	 * 
	 * @return required arm angle
	 */
	public static double calcAngle(double height) {
		return Math.PI - Math.asin(height / resultantLength());
	}

	public static double resultantLength() {
		return Math.sqrt(Math.pow(currentBeltExtension(), 2) + Math.pow(assumedSupportArmLength, 2));
	}
}
