package commands;

/**
 * This class is just a placeholder who's bark is worse than its bite
 * @author Shota
 *
 */
public class MidLevelCommand {
	
	String action;
	
	public MidLevelCommand(String s){
		action = s;
	}
	
	@Override
	public String toString(){
		return action;
	}
}
