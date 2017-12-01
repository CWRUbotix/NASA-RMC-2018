package test;

import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.MidLine;
import main.java.com.cwrubotix.glennifer.automodule.ModifiedAStar;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.PathFinder;
import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;

import java.util.Arrays;

import javax.swing.JOptionPane;

import javafx.application.Application;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.VBox;
import javafx.scene.control.TextArea;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Rectangle;
import javafx.scene.Scene;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.stage.Stage;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.input.MouseEvent;
import javafx.event.EventHandler;
import javafx.event.ActionEvent;
import javafx.geometry.Pos;

/**
 * 
 * Simulator for different path plan algorithms for autonomy operation
 *
 */
public class PathPlanSimulator{
	
    /** Array of Positions that represent obstacles in the arena*/
    private Position[] obstacles = new Position[6];
    /** Array of Positions that represent obstacles we found*/
    private Position[][] obstaclesFound = new Position[4][6];
    /** The destination of the path*/
    private final Position destination;
    /** 
     * Array of Positions which represents locations of robots.
     * <p>We will have one robot per path</p>
     */
    private Position[] robots = new Position[4];
    /** Array of paths created by different algorithms.*/
    private Path[] paths = new Path[4];
    private PathFinder<? extends PathFindingAlgorithm>[] finders;
    
    /** Array whose element checks whether each robot arrived at destination*/
    private boolean[] robotArrived = new boolean[4];
    
    /*Constants:*/
    /** Stores Max straight speed of the robot. Unit: m/s*/
    private static final float MAX_STRAIGHT_SPEED = 3.32F;
    /** Stores Max turning speed of the robot. Unit: rad/s*/
    private static final float MAX_TURNING_SPEED = (float)Math.PI;
    /** Stores robot's width in meters*/
    /** Stores maximum diameter of obstacles in meters*/
    private static final float OBSTACLE_SIZE = 0.3F;
    /** Stores the maximum reliable kinect range in meters*/
    private static final float KINECT_RANGE = 2.0F;
    /** 
     * Stores the y direction length of obstacle area of the arena in meters
     * <p>Actual positions in the arena : y = 1.5m ~ 4.44m</p>
     */
    private static final float OBSTACLE_AREA_HEIGHT = 2.94F;
    /** 
     * Stores the y direction length of obstacle safe area of the arena in meters
     * <p>Actual positions in the arena : y = 0.0m ~ 1.5m</p>
     */
    private static final float SAFE_AREA_HEIGHT = 1.5F;
    
    
    
    //add more fields/constants if necessary.
    
    /*
     * For paths array:
     *   I will just assign indexes to each path created by different algorithms to keep things organized.
     *   Let me know if you come up with better idea :)
     *   index 0: path created by midLine algorithm
     *   index 1: path created by modifiedAStar algorithm.
     *   index 2: path created by arcPath algorithm
     *   index 3: path created by Dijkstra algorithm 
     */
    
    /**
     * Constructor for simulator
     * <p>
     * Sets up initial locations of robots, obstacles generated, and destinations along with proper GUI setup.
     * </p>
     * @param initialPos the start point of simulation
     * @param destination the destination of simulation
     */
    public PathPlanSimulator(Position initialPos, Position destination){
	for(int i = 0; i < robots.length; i ++){
	    robots[i] = (Position) initialPos.clone();
	}
	this.destination = destination;
	Arrays.fill(robotArrived, false);
	
	finders = new PathFinder[4];
	finders[0] = new PathFinder<MidLine>(new MidLine(initialPos, destination), initialPos, destination);
	finders[1] = new PathFinder<ModifiedAStar>(new ModifiedAStar(initialPos, destination), initialPos, destination);
	//finders[2] = new PathFinder<ArcPath>(new ArcPath()); //Setting up arcPath
	//finders[3] = new PathFinder<>(new ); //Setting up gridAStar
	for(int i = 0; i < paths.length - 2; i ++)
	    paths[i] = finders[i].getAlgorithm().computePath(initialPos, destination);
	generateObstacles();
    }
    
    public Position[] getObstacles(){
	return obstacles;
    }
    
    public Position[] getRobots(){
	return robots;
    }
    
    public Path[] getPaths(){
	return paths;
    }
    
    /**
     * Generates obstacles within the arena.
     * <p>
     * Each obstacle is an instance of the position class and fits inside the arena.
     * <p>
     * This method ensures the obstacle fits in the arena by taking into account the diameter
     * and ensuring that the center of the obstacle is in a valid location. 
     * 
     * @author Tyler Thieding
     */
    private void generateObstacles(){
	float validObstacleWidthLength = Position.ARENA_WIDTH() - OBSTACLE_SIZE;
	float validObstacleHeightLength = OBSTACLE_AREA_HEIGHT - OBSTACLE_SIZE;
	for(int i=0; i<obstacles.length; i++) {
	    float newObstacleX = (float) (OBSTACLE_SIZE + validObstacleWidthLength*Math.random() - Position.ARENA_WIDTH() / 2);
	    float newObstacleY = (float) (OBSTACLE_SIZE + SAFE_AREA_HEIGHT + validObstacleHeightLength*Math.random());
	    Position newObstacle = new Position(newObstacleX, newObstacleY, 0, 0);
	    obstacles[i] = newObstacle;
	}
    }
    
    /**
     * Changes position of given obstacle with given coordinates.
     * @param obstacle the obstacle whose position is being modified
     * @param x_pos the new x coordinate
     * @param y_pos the new y coordinate
     * @return false if given coordinates are invalid as an obstacle.
     */
    private boolean modifyObstacle(Position obstacle, float x_pos, float y_pos){
	if(y_pos > SAFE_AREA_HEIGHT && y_pos < SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT){ //if inside obstacle area
	    if(!obstacle.setX(x_pos) || !obstacle.setY(y_pos)){
		return false;
	    }
	    else{
		return true;
	    }
	}
	return false;
    }
    

    private Position findNextEncounter(int robot){
	Arrays.sort(obstacles, Position.getComparatorByDistTo(robots[robot]));
	Position foundObstacle = null;
	for(Position obs : obstacles){
	    boolean found = true;
	    if(robots[robot].getDistTo(obs) <= KINECT_RANGE){
		for(int i = 0; i < obstaclesFound[robot].length; i ++){
			if(obs.equals(obstaclesFound[robot][i]))
			    found = false; 
	    	}
		if(found){
		    foundObstacle = obs;
		    break;
	    	}
	    }
	}
	return foundObstacle;
    }
    
    /*NOT WORKING*/
    private void moveRobot(int robot){
    	Position currentpos = robots[robot];
    	finders[robot].setCurrentPos(currentpos);
	paths[robot] = finders[robot].getPath();
	int subgoal = 1;
	currentpos.setAngle(paths[robot].getPoint(0).getAngleTurnTo(paths[robot].getPoint(1)));
	float increment = 0.05F;
	while(!currentpos.equals(destination)){
	    Position obsFound = findNextEncounter(robot);
	    if(obsFound != null){
		finders[robot].registerObstacle(new Obstacle(obsFound.getX(), obsFound.getY(), OBSTACLE_SIZE));
		paths[robot] = finders[robot].getPath();
		if(subgoal < paths[robot].length())
		    currentpos.setAngle(currentpos.getAngleTurnTo(paths[robot].getPoint(subgoal)));
	    }
	    if(subgoal < paths[robot].length() && currentpos.getDistTo(paths[robot].getPoint(subgoal)) < increment){
		currentpos = (Position) paths[robot].getPoint(subgoal).clone();
		subgoal++;
	    }
	    else{
		if(
		!currentpos.setX((float) (currentpos.getX() + increment * Math.cos(currentpos.getAngle())))||
		!currentpos.setY((float) (currentpos.getY() + increment * Math.sin(currentpos.getAngle()))))
		    throw new RuntimeException("Trouble moving robot");
	    }
	    finders[robot].setCurrentPos(currentpos);
	}
    }
    
    public void runSimulation(){
	for(int i = 1; i < robots.length - 2; i++){
	    moveRobot(i);
	}
    }
    
    ///////////////////////////////////////////////////////////GUIS////////////////////////////////////////////////////////////	

    /**
     * GUI display for simulator
     * @author Seohyun Jung
     *
     */
    
    public static class GUI extends Application{
	
	private PathPlanSimulator simulator;
	private Stage primaryStage;
	private BorderPane pane;
	private Pane arena;
	private boolean modificationAllowed = true;
	private boolean simulationRun = false;
	private TextArea[] result;
	private Group[] paths;
	
	public void start(Stage primaryStage){
	    this.primaryStage = primaryStage;
	    setUpSimulation();
	    setUp();
	    primaryStage.setResizable(false);
	    primaryStage.show();
	}
	
	private void setUpSimulation(){
	    String[] args =  new String[getParameters().getRaw().size()];
	    args = getParameters().getRaw().toArray(args);
	    Position start = new Position(0.0F, 0.0F, 0.0, 0.0F);
	    Position destination = new Position(0.0F, 0.0F, 0.0, 0.0F);
	    if(!start.setX(Float.parseFloat(args[0])) || !start.setY(Float.parseFloat(args[1]))
		    || !destination.setX(Float.parseFloat(args[2])) || !destination.setY(Float.parseFloat(args[3])))
		throw new RuntimeException("Given coordinates are invalid to set up simulation");
	    simulator = new PathPlanSimulator(start, destination);
	}
	
	private void setUp(){
	    primaryStage.setTitle("PathPlanSimulator");
	    primaryStage.setHeight(Position.ARENA_HEIGHT() * 100 + 100);
	    primaryStage.setWidth(600 + Position.ARENA_WIDTH()*100);
	    pane = new BorderPane();
	    arena = new Pane();
	    arena.setPrefSize(Position.ARENA_WIDTH() * 100, Position.ARENA_HEIGHT() * 100);
	    setUpResultDisplay();
	    drawArena();
	    Button startButton = new Button("START");
	    startButton.setAlignment(Pos.BOTTOM_CENTER);
	    startButton.setOnAction(new EventHandler<ActionEvent>(){
		public void handle(ActionEvent e){
		    if(simulationRun)
			return;
		    modificationAllowed = false;
		    simulationRun = true;
		    simulator.runSimulation();
		    displayPaths();
		}
	    });
	    VBox left = new VBox();
	    left.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    left.getChildren().addAll(result[0], result[1]);
	    VBox right = new VBox();
	    right.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    right.getChildren().addAll(result[2], result[3]);
	    pane.setLeft(left);
	    pane.setRight(right);
	    pane.setCenter(arena);
	    pane.setBottom(startButton);
	    primaryStage.setScene(new Scene(pane));
	}
	
	private void drawArena(){
	    Line line1 = new Line(0.0, getDisplayY(SAFE_AREA_HEIGHT), Position.ARENA_WIDTH() * 100, getDisplayY(SAFE_AREA_HEIGHT));
	    Line line2 = new Line(0.0, getDisplayY(SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT), Position.ARENA_WIDTH() * 100, getDisplayY(SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT));
	    Line line3 = new Line(getDisplayX(0), 0.0, getDisplayX(0), Position.ARENA_HEIGHT() * 100);
	    Rectangle bin = new Rectangle(getDisplayX(0) - 50, getDisplayY(0) - 25, 100, 50);
	    line1.setFill(Color.LIGHTGRAY);
	    line2.setFill(Color.LIGHTGRAY);
	    line3.setFill(Color.LIGHTGOLDENRODYELLOW);
	    bin.setFill(Color.BLUE);
	    arena.getChildren().addAll(line1, line2, line3, bin);
	    setUpObstacles();
	}
	
	private void setUpObstacles(){
	    for(Position p : simulator.getObstacles()){
		Circle obstacle = new Circle(getDisplayX(p.getX()) - 15, getDisplayY(p.getY()) - 15, 15, Color.DARKGREY);
		obstacle.setOnMouseClicked(new EventHandler<MouseEvent>(){
		    final Position obs = p;
		    public void handle(MouseEvent e){
			if(!modificationAllowed)
			    return;
			float newX, newY;
			try{
			    newX = Float.parseFloat(JOptionPane.showInputDialog("Enter new X_Pos coordinate"));
			    newY = Float.parseFloat(JOptionPane.showInputDialog("Enter new Y_Pos coordinate"));
			}
			catch(NumberFormatException error){
			    JOptionPane.showMessageDialog(null, "Wrong input, modification aborted.");
			    return;
			}
			if(!simulator.modifyObstacle(obs, newX, newY))
			    JOptionPane.showMessageDialog(null, "Wrong input, modification aborted.");
			else {
			    ((Circle)e.getSource()).relocate(getDisplayX(newX), getDisplayY(newY));
			}
		    }
		});
		arena.getChildren().add(obstacle);
	    }
	}
	
	private void displayPaths(){
	    Path[] path = simulator.getPaths();
	    paths = new Group[4];
	    for(int i = 0; i < paths.length; i++){
		paths[i] = new Group();
		for(int pos = 0; pos < path[i].length(); i ++){
		    Position point = path[i].getPoint(pos);
		    Circle circle = new Circle(getDisplayX(point.getX()) - 2.5, getDisplayY(point.getY()) - 2.5, 2.5);
		    circle.setFill(Color.RED);
		    paths[i].getChildren().add(circle);
		    if(pos != path[i].length() - 1){
			Position next = path[i].getPoint(pos + 1);
			Line line = new Line(getDisplayX(point.getX()), getDisplayY(point.getY()), getDisplayX(next.getX()), getDisplayY(next.getY()));
			line.setFill(Color.GREEN);
			paths[i].getChildren().add(line);
		    }
		}
	    }
	    arena.getChildren().addAll(paths);
	}
	
	private void setUpResultDisplay(){
	    result = new TextArea[4];
	    for(int i = 0; i < result.length; i++){
		result[i] = new TextArea();
		result[i].setEditable(false);
		result[i].setPrefSize(300, Position.ARENA_HEIGHT() * 50);
	    }
	    result[0].appendText("MidLine Algorithm:\n");
	    result[1].appendText("ModifiedAStar Algorithm:\n");
	    result[2].appendText("ArcPath Algorithm: \n");
	    result[3].appendText("GridAStar Algorithm: \n");
	}
	
	private double getDisplayX(float x_pos){
	    return (double)((x_pos + Position.ARENA_WIDTH() / 2) * 100);
	}
	
	private double getDisplayY(float y_pos){
	    return (double)(y_pos * 100);
	}
	
	private static void launchWrap(String[] args){
	    Application.launch(args);
	}
    }

    /////////////////////////////////////////////////////////////GUI ENDS//////////////////////////////////////////////////////

    public static void main(String[] args){
	GUI.launchWrap(args);
    }
    
}
