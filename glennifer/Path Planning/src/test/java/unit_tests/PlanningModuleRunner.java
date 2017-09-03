package unit_tests;

import java.util.InputMismatchException;
import java.util.Queue;
import java.util.Scanner;

import commands.HighLevelCommand;
import commands.MidLevelCommand;
import planning_modules.Deposition;
import planning_modules.Excavation;
import planning_modules.Locomotion;

public class PlanningModuleRunner {

	Queue<MidLevelCommand> MidQueue;
	Queue<HighLevelCommand> HighQueue;

	public static void main(String[] args) {

		boolean run = true;
		int id_counter = 0;

		while (run) {
			id_counter++;
			
			System.out.println("\nPlease enter the number of the type of high level command you wish to execute"
					+ "\n(1) Locomotion\n(2) Deposition\n(3) Excavation\n(4) Quit");

			int inputInt = inputErrorCheck(new int[] { 1, 2, 3, 4 });

			if (inputInt == 2) {
				System.out.println(
						"\n----Deposition Mode----\nPlease enter type of dump:\n(1) Dump All \n(2) Specific Duration\n(3) Specific Remaining Amount\n"
								+ "Current bin position: 0\nCurrent bin load: 60 kg");

				inputInt = inputErrorCheck(new int[] { 1, 2, 3 });

				if (inputInt == 1) {
					printQueue(Deposition.receiveCommand(new HighLevelCommand(id_counter, 0f)));
				} else if (inputInt == 2) {
					System.out.println("\nPlease enter the desired duration of the dump:\n");
					float fracDuration = floatInput();
					int intDuration = (int) fracDuration;
//					fracDuration -= intDuration;

					printQueue(Deposition.receiveCommand(new HighLevelCommand(id_counter, intDuration)));

				} else if (inputInt == 3) {
					System.out.println("\nPlease enter the desired remaining amount in the bin:\n");
					float amount = floatInput();

					printQueue(Deposition.receiveCommand(new HighLevelCommand(id_counter, amount)));
				}

			} else if (inputInt == 1) {
				System.out.println(
						"\nJason hasn't finished his locomotion code yet so I'm just outputting a sample of random turning and moving commands");

				printQueue(Locomotion.receiveCommand(new HighLevelCommand(id_counter, 1, 2, 3)));

			}

			else if (inputInt == 3) {
				System.out.println(
						"\n----Excavation Mode----\nPlease enter type of Excavation:\n(1) Specific Duration\n(2) Specific Amount in Bin\n"
								+ "Current bin position: 0\nCurrent bin load: 0 kg");

				inputInt = inputErrorCheck(new int[] { 1, 2, 3 });

				if (inputInt == 1) {
					System.out.println("\nPlease enter the desired depth of the excavation:\n");
					float depth = floatInput();

					System.out.println("\nPlease enter the desired duration of the excavation:\n");
					float fracDuration = floatInput();
					int intDuration = (int) fracDuration;
					//fracDuration -= intDuration;

					printQueue(Excavation.receiveCommand(new HighLevelCommand(id_counter, depth, intDuration)));

				} else if (inputInt == 2) {
					System.out.println("\nPlease enter the desired depth of the excavation:\n");
					float depth = floatInput();

					System.out.println("\nPlease enter the desired remaining amount in the bin:\n");
					float amount = floatInput();

					printQueue(Excavation.receiveCommand(new HighLevelCommand(id_counter, depth, amount)));
				}
			}
			
			else if (inputInt == 4){
				run = false;
			}
		}

	}

	/**
	 * Checks if the given integer is valid. Asks for another entry if invalid
	 * 
	 * @param validEntries
	 *            The list of valid integers
	 * @return Returns the given integer if it is valid
	 */
	public static int inputErrorCheck(int[] validEntries) {
		int inputInt = 0;
		boolean Error = true;

		Scanner scan = new Scanner(System.in);

		while (Error) {
			inputInt = scan.nextInt();

			for (int a : validEntries) {
				Error = inputInt == a ? false : true;
				if (!Error)
					break;
			}

			if (Error) {
				System.err.println("Invalid entry, please enter one of the following: ");
				for (int b : validEntries)
					System.err.print("'" + b + "' ");
			}
		}

		return inputInt;
	}

	/**
	 * Ensures that the user input is a float
	 * 
	 * @return the float if it is a valid entry
	 */
	public static float floatInput() {
		Scanner scan = new Scanner(System.in);

		float output = 0;
		boolean Error = true;

		while (Error) {
			try {
				output = scan.nextFloat();
			} catch (InputMismatchException e) {
				System.err.println("Invalid entry, please enter a float number\n");
				Error = false;
			}

			Error = !Error ? true : false;
		}

		return output;

	}

	/**
	 * Prints out the MidLevelCommands in the given Queue;
	 */
	public static void printQueue(Queue<MidLevelCommand> q) {
		System.out.println("\nCommand List:");
		int counter = 1;
		for (MidLevelCommand c : q) {
			System.out.println(counter + ") " + c.toString());
			counter++;
		}
	}

}
