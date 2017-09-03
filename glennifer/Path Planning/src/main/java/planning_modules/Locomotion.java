package planning_modules;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import commands.HighLevelCommand;
import commands.MidLevelCommand;

/**
 * This class will return the list of commands required to carry out a high
 * level locomotion command.
 * 
 * @author Jason
 *
 */
public class Locomotion {

	/**
	 * this is just a temporary placeholder for the function that will return
	 * the robot's actual current angle
	 */
	public static float initialAngle = 0;

	private static class Vector {
		float angle;
		float distance;

		Vector(float angle, float distance) {
			this.angle = angle;
			this.distance = distance;
		}
	}

	public static Queue<MidLevelCommand> receiveCommand(HighLevelCommand highCommand) {
		Queue<MidLevelCommand> output = new LinkedList<MidLevelCommand>();

		// ArenaObjectMap is a placeholder for
		// Vector[] v = calculateVectors(highCommand.getX(), highCommand.getY(),
		// ArenaObjectMap);
		if (highCommand.getType() == 1) {
			
			Vector[] v = calculateVectors((int) highCommand.getData(0), (int) highCommand.getData(1));
			float currentAngle = initialAngle;
			for (Vector c : v) {
				output.add(turnToFaceDirection(currentAngle, c.angle));
				output.add(moveForVectorDistance(c));
				currentAngle = c.angle;
			}

			return output;
		}
		else{
			System.err.println("Wrong type of Command");
			return null;
		}
	}

	public static Vector[] calculateVectors(float endX, float endY) {
		// Insert Waypoint finding algorithm stuff in here, i.e. Astar, but for
		// now:

		Random randy = new Random();
		int numVectors = randy.nextInt(5) + 1;
		Vector[] output = new Vector[numVectors];

		for (int i = numVectors - 1; i > -1; i--) {
			output[i] = new Vector(randy.nextFloat() * 360, randy.nextFloat() * 100);
		}

		return output;
	}

	public static MidLevelCommand turnToFaceDirection(float currentAngle, float targetAngle) {
		return new MidLevelCommand("Turn from " + currentAngle + " degrees to " + targetAngle);
	}

	public static MidLevelCommand moveForVectorDistance(Vector v) {
		return new MidLevelCommand("Move " + v.distance + " meters");
	}

	public static float getCurrentAngle() {
		return initialAngle;
	}
}
