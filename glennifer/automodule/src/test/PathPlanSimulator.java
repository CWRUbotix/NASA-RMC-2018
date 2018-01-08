package test;

import main.java.com.cwrubotix.glennifer.automodule.MidLine;
import main.java.com.cwrubotix.glennifer.automodule.ModifiedAStar;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;
import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.astargrid.AStarPathFinder;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.PathFinder;
import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;

import java.util.ArrayList;

import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.TextArea;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Rectangle;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

/**
 * 
 * Simulator for different path plan algorithms for autonomy operation
 *
 */
public class PathPlanSimulator{
	
    /** Array of Positions that represent obstacles in the arena*/
    private Obstacle[] obstacles = new Obstacle[6];
    /** Array of paths created by different algorithms.*/
    private Path[] paths = new Path[3];
    /** Array of PathFinders that represents each algorithm*/
    private PathFinder<?>[] finders = new PathFinder[3];
    /** The initial position of the robot*/
    private Position initialPos;
    /** The destination of the robot*/
    private Position destination;
    /** Stores errorMessages created during simulation*/
    private StringBuilder errorMessages = new StringBuilder();
    
    /*Constants:*/
    /** Stores Max straight speed of the robot. Unit: m/s*/
    private static final float MAX_STRAIGHT_SPEED = 3.32F;
    /** Stores Max turning speed of the robot. Unit: rad/s*/
    private static final float MAX_TURNING_SPEED = (float)Math.PI;
    /** Stores maximum radius of obstacles in meters*/
    private static final float OBSTACLE_SIZE = 0.15F;
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
    
    /**
     * Constructor for simulator
     * <p>
     * Sets up initial locations of robots, obstacles generated, and destinations along with proper GUI setup.
     * </p>
     * @param initialPos the start point of simulation
     * @param destination the destination of simulation
     */
    public PathPlanSimulator(Position initialPos, Position destination){
	    this.initialPos = initialPos;
	    this.destination = destination;
	    generateObstacles();
	    finders[0] = new PathFinder<MidLine>(new MidLine(), initialPos, destination);
	    finders[1] = new PathFinder<ModifiedAStar>(new ModifiedAStar(), initialPos, destination);
	    finders[2] = new AStarPathFinder(initialPos, destination, 5e-2);
	    for(int i = 0; i < paths.length; i ++){
	      if(finders[i] == null)
		      continue;
	      paths[i] = finders[i].getPath();
	    }
    }
    
    /**
     * Returns the start point of the simulation
     * @return the start point of the simulation
     */
    public Position getInitialPos(){
	    return initialPos;
    }
    
    /**
     * Returns the destination point of the simulation
     * @return the destination point of the simulation
     */
    public Position getDestination(){
	    return destination;
    }
    
    /**
     * Returns the list of the obstacles
     * @return the list of the obstacles
     */
    public Obstacle[] getObstacles(){
	    return obstacles;
    }
    
    /**
     * Returns the list of paths created during simulation
     * @return
     */
    public Path[] getPaths(){
	    return paths;
    }
    
    /**
     * Returns the StringBuilder that stores all error messages thrown during simulation
     * @return the StringBuilder that stores all error messages thrown during simulation
     */
    public StringBuilder getErrorMessages(){
	    return errorMessages;
    }
    
    /**
     * Returns the list of the path plan algorithms
     * @return the list of the path plan algorithms
     */
    public PathFinder<?>[] getFinders(){
	return finders;
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
	float validObstacleWidthLength = Position.ARENA_WIDTH() - (OBSTACLE_SIZE * 2);
	float validObstacleHeightLength = OBSTACLE_AREA_HEIGHT - (OBSTACLE_SIZE * 2);
	for(int i=0; i<obstacles.length; i++) {
	    float newObstacleX = (float) (OBSTACLE_SIZE + validObstacleWidthLength*Math.random() - Position.ARENA_WIDTH() / 2);
	    float newObstacleY = (float) (OBSTACLE_SIZE + SAFE_AREA_HEIGHT + validObstacleHeightLength*Math.random());
	    Obstacle newObstacle = new Obstacle(newObstacleX, newObstacleY, OBSTACLE_SIZE / 2);
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
    private boolean modifyObstacle(Obstacle obstacle, float x_pos, float y_pos){
	float previousX = obstacle.getX();
	float previousY = obstacle.getY();
	if(y_pos > SAFE_AREA_HEIGHT && y_pos < SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT){ //if inside obstacle area
	    if(!obstacle.setX(x_pos) || !obstacle.setY(y_pos)){
		obstacle.setX(previousX);
		obstacle.setY(previousY);
		return false;
	    }
	    else{
		return true;
	    }
	}
	return false;
    }
    
    /**
     * Helper method to run simulation algorithm by algorithm
     * @param robot int value that indicates which algorithm to run
     */
    private void moveRobot(int robot){
	/* 
	 * MidLine does not work :( Causes infinite loop too often + rarely creates pretty path. 
	 * Feel Free to comment the line below to see what happens
	 */
	if(robot == 0) throw new PathFindingAlgorithm.AlgorithmFailureException(); 
	boolean arrived = false;
	PathFinder<?> finder = finders[robot];
	Path path = finder.getPath();
	Position currentPos = path.getPoint(0);
	int progress = 0;
	ArrayList<Obstacle> obstacles = new ArrayList<>(6); // Copying obstacles in to new list
	for(int i = 0; i < 6; i++){
	    obstacles.add(getObstacles()[i]);
	}
	while(!arrived){ //While the robot is on transit
	    obstacles.sort(Position.getComparatorByDistTo(currentPos)); //sort the obstacles by proximity to the robot
	    Obstacle encountered = null;
	    for(Obstacle obs : obstacles){
		Position temp = getEncounter(path, progress, obs, robot); //returns position if the robot sees the obstacle
		if(temp != null){
		    currentPos = temp; // where robot is currently standing
		    finder.setCurrentPos(currentPos); //
		    finder.registerObstacle(obs);     // Running path plan algorithm
		    path = finder.getPath();          //
    		    encountered = obs;
		    break;			      // Break out of the loop so that the simulation handles one obstacle at a time
		}
	    }
	    if(encountered != null){
		obstacles.remove(encountered);// remove the obstacle already found from the list
		progress = 0;		      // start evaluating from the beginning making sure the newly created path does not run into obstacle
		/*
		 *NOTE: setting progress to 0 will make sure that created path is valid, 
		 *but the path will slightly differ from how robot will actually move during run 
		 */
	    }
	    else{
		if(progress == path.length() - 2) // Reached end point
		    arrived = true;
		else
		    progress++; // Increment progress if not yet reached end point
	    }
	}
	paths[robot] = path; //setting path field
    }
    
    /**
     * Returns the Position where the robot sees the obstacle given if the robot encounters the obstacle between
     * current position and the next position in the path.
     * @param path the path the robot is currently on
     * @param progress indicates the index of robot's position within the current path
     * @param obs the obstacle being evaluated
     * @return the Position where the robot sees the obstacle given if the robot encounters the obstacle.
     */
    private Position getEncounter(Path path, int progress, Obstacle obs, int robot){
	/*For AStarGrid*/
	if(robot == 2)
	    return gridGetEncounter(path.getPoint(progress), obs);

	Position p1 = path.getPoint(progress); //current position
	Position p2 = path.getPoint(progress + 1); //next position
	float x1 = p1.getX(); // current position
	float y1 = p1.getY(); // coordinate
	float x2 = p2.getX(); // next position
	float y2 = p2.getY(); // coordinate
	float cx = obs.getX();// center of the obstacle
	float cy = obs.getY();// coordinate
		
	/*Getting the equation of line formed by current position and next position*/
	double slope = (y2 - y1)/(x2 - x1);
	double y_intercept = y1 - slope * x1;
	
	/*coefficients of quadratic equation formed by line = circle(the range that robot can see the obstacle centered at obstacle)*/
	double a = 1 + Math.pow(slope, 2);
	double b = -2 * (cx - slope * (y_intercept - cy));
	double c = (Math.pow(cx, 2) + Math.pow((y_intercept - cy), 2)) - Math.pow(KINECT_RANGE, 2);
	
	double check = Math.pow(b, 2) - 4 * a * c; // to check whether quadratic equation has roots

	if(check < 0){ // quadratic equation does not have any real solutions
	    return null;
	}
	
	/*Solving the system of equations*/
	float rx1 = (float)((-b + Math.sqrt(check)) / (2 * a));
	float ry1 = (float)(slope * rx1 + y_intercept);
	float rx2 = (float)((-b - Math.sqrt(check)) / (2 * a));
	float ry2 = (float)(slope * rx2 + y_intercept);
	
	Position r1 = new Position(rx1, ry1);
	Position r2 = new Position(rx2, ry2);
	
	/*Checking whether the points calculated are between current position and next position*/
	if(rx1 < Math.min(x1, x2) || rx1 > Math.max(x1, x2) || Double.isNaN(rx1) || Double.isNaN(ry1))
	    r1 = null;
	if(rx2 < Math.min(x1, x2) || rx2 > Math.max(x1, x2) || Double.isNaN(rx2) || Double.isNaN(ry2))
	    r2 = null;
	
	/* Returning appropriate position.*/
	if(r1 == null && r2 == null){ // If both are out of range
	    return null;
	}
	else if(r1 == null){ // when one of them are within range
	    return r2;
	}
	else if(r2 == null){
	    return r1;
	}
	else{ //if both are within range return whatever is closer to the current robot position
	    if(r1.getDistTo(p1) < r2.getDistTo(p2)){
		return r1;
	    }
	    else{
		return r2;
	    }
	}
    }
    
    /**
     * FOR ASTARGRID ONLY
     * For AStarGrid, the distances between consecutive positions inside the path are too small to use getEncounter method above.
     * Therefore, this method checks whether the robot sees the obstacle at the current position and returns the position if it does, 
     * and null if it does not
     * @param currentPos current position of the robot
     * @param obs the obstacle being evaluated
     * @return the currentPos if the robot sees the obstacle, null if it does not
     */
    private Position gridGetEncounter(Position currentPos, Obstacle obs){
	/*The distance squared value from currentPos to center of the Obstacle*/
	double check = Math.pow(currentPos.getX() - obs.getX(), 2) + Math.pow(currentPos.getY() - obs.getY(), 2);
	/*If the distance squared is less than the Kinect range squred, the robot saw the obstacle.*/
	if(check < Math.pow(KINECT_RANGE, 2))
	    return currentPos;
	else
	    return null;
    }
    
    /**
     * Runs simulation.
     */
    public void runSimulation(){
	for(int i = 0; i < finders.length; i++){
	    try{
		moveRobot(i);
	    }
	    catch(RuntimeException e){ //When algorithm fails
		String failedOne = "";
		switch(i){
		case 0: failedOne = "MidLine"; break;
		case 1: failedOne = "ModifiedAStar"; break;
		case 2: failedOne = "AStarGrid"; break;
		}
		errorMessages.append(failedOne + ": \n" + e.toString() + "\n\n"); // Store what caused failure
		paths[i] = null; //Marking the path that failed.
	    }
	}
    }
    
    ///////////////////////////////////////////////////////////GUIS////////////////////////////////////////////////////////////	

    /**
     * GUI display for simulator
     * @author Seohyun Jung
     *
     */
    
    public static class GUI extends Application{
	
	/** Stores the simulator instance*/
	private PathPlanSimulator simulator;
	/** Stores main window of the simulation*/
	private Stage primaryStage;
	/** the main structure of the window*/
	private BorderPane pane = new BorderPane();
	/** area that represents competition arena*/
	private Pane arena = new Pane();
	/** Stores whether modification on obstacles are allowed*/
	private boolean modificationAllowed = true;
	/** Stores whether simulation was performed*/
	private boolean simulationRun = false;
	/** Stores the obstacle being modified by user*/
	private Obstacle beingModifiedObs = null;
	/** Stores the Circle that represents obstacle being modified by user*/
	private Circle beingModifiedCir = null;
	/** list of TextArea instances that displays result*/
	private TextArea[] result = new TextArea[3];
	/** TextArea that displays error messages*/
	private TextArea errorMessages = new TextArea();
	/** list of Group instances which each will store each path graphics*/
	private Group[] paths = new Group[3];
	
	@Override
	public void start(Stage primaryStage){
	    this.primaryStage = primaryStage;
	    setUpSimulation();
	    setUp();
	    primaryStage.setResizable(false);
	    primaryStage.show();
	}
	
	/**
	 * Sets up simulation based on command line arguments
	 */
	private void setUpSimulation(){
	    String[] args =  new String[getParameters().getRaw().size()];
	    args = getParameters().getRaw().toArray(args);
	    Position start = new Position(0.0F, 0.0F, Math.PI / 2, 0.0F);
	    Position destination = new Position(0.0F, 0.0F);
	    if(!start.setX(Float.parseFloat(args[0])) || !start.setY(Float.parseFloat(args[1]))
		    || !destination.setX(Float.parseFloat(args[2])) || !destination.setY(Float.parseFloat(args[3])))
		throw new RuntimeException("Given coordinates are invalid to set up simulation");
	    simulator = new PathPlanSimulator(start, destination);
	}
	
	/**
	 * Sets up general GUI structure
	 */
	private void setUp(){
	    primaryStage.setTitle("PathPlanSimulator");
	    primaryStage.setHeight(Position.ARENA_HEIGHT() * 100 + 100);
	    primaryStage.setWidth(600 + Position.ARENA_WIDTH()*100);
	    
	    setUpArena();
	    setUpResultDisplay();
	    Button startButton = new Button("START");
	    startButton.setAlignment(Pos.BOTTOM_CENTER);
	    startButton.setOnAction(new EventHandler<ActionEvent>(){
		public void handle(ActionEvent e){
		    if(simulationRun)
			return;
		    modificationAllowed = false;
		    simulator.runSimulation();
		    displayPaths();
		    updateResult();
		    simulationRun = true;
		    errorMessages.appendText(simulator.getErrorMessages().toString());
		}
	    });
	    
	    Button reRunAlgorithmButton = new Button("Re-run algorithm");
	    reRunAlgorithmButton.setOnAction(new EventHandler<ActionEvent>(){
		public void handle(ActionEvent e){
		    if(!simulationRun)
			return;
		    for(int i = 0; i < simulator.getPaths().length; i ++){
			if(simulator.getPaths()[i] != null){
			    simulator.getFinders()[i].runAlgorithm();
			    simulator.getPaths()[i] = simulator.getFinders()[i].getPath();
			}
		    }
		    displayPaths();
		    updateResult();
		}
	    });
	    
	    errorMessages.setEditable(false);
	    errorMessages.setPrefSize(300, Position.ARENA_HEIGHT() * 50);
	    VBox left = new VBox();
	    left.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    left.getChildren().addAll(result[0], result[1], startButton);
	    VBox right = new VBox();
	    right.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    right.getChildren().addAll(result[2], errorMessages, reRunAlgorithmButton);
	    left.setAlignment(Pos.CENTER);
	    right.setAlignment(Pos.CENTER);
	    pane.setLeft(left);
	    pane.setRight(right);
	    pane.setCenter(arena);
	    primaryStage.setScene(new Scene(pane));
	}
	
	/**
	 * Sets up arena graphics
	 */
	private void setUpArena(){
	    arena.setPrefSize(Position.ARENA_WIDTH() * 100, Position.ARENA_HEIGHT() * 100);
	    arena.setOnMouseReleased(new EventHandler<MouseEvent>(){

		@Override
		public void handle(MouseEvent event) {
		    if(beingModifiedObs == null)
			return;
		    float x = getArenaX(event.getX());
		    float y = getArenaY(event.getY());
		    if(simulator.modifyObstacle(beingModifiedObs, x, y)){
			beingModifiedCir.setCenterX(getDisplayX(x));
			beingModifiedCir.setCenterY(getDisplayY(y));
		    }
		    beingModifiedObs = null;
		    beingModifiedCir = null;
		}
		
	    });
	    
	    drawArena();
	}
	
	/**
	 * Detailed arena structure graphics setup
	 */
	private void drawArena(){
	    Line line1 = new Line(0.0, getDisplayY(SAFE_AREA_HEIGHT), Position.ARENA_WIDTH() * 100, getDisplayY(SAFE_AREA_HEIGHT));
	    Line line2 = new Line(0.0, getDisplayY(SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT), Position.ARENA_WIDTH() * 100, getDisplayY(SAFE_AREA_HEIGHT + OBSTACLE_AREA_HEIGHT));
	    Line line3 = new Line(getDisplayX(0), 0.0, getDisplayX(0), Position.ARENA_HEIGHT() * 100);
	    Rectangle bin = new Rectangle(getDisplayX(0) - 50, getDisplayY(0) - 25, 100, 50);
	    Circle start = new Circle(getDisplayX(simulator.getInitialPos().getX()), getDisplayY(simulator.getInitialPos().getY()), 20);
	    Circle end = new Circle(getDisplayX(simulator.getDestination().getX()), getDisplayY(simulator.getDestination().getY()), 20);
	    line1.setFill(Color.LIGHTGRAY);
	    line2.setFill(Color.LIGHTGRAY);
	    line3.setFill(Color.LIGHTGOLDENRODYELLOW);
	    start.setFill(Color.RED);
	    end.setFill(Color.RED);
	    bin.setFill(Color.BLUE);
	    arena.getChildren().addAll(line1, line2, line3, bin, start, end);
	    setUpObstacles();
	}
	
	/**
	 * Displays obstacles created
	 */
	private void setUpObstacles(){
	    for(Obstacle p : simulator.getObstacles()){
		Circle obstacle = new Circle(getDisplayX(p.getX()), getDisplayY(p.getY()), 15, Color.DARKGREY);
		obstacle.setOnMousePressed(new EventHandler<MouseEvent>(){
		    final Obstacle obs = p;
		    final Circle cir = obstacle;
		    public void handle(MouseEvent e){
			if(!modificationAllowed){ 
			    beingModifiedObs = null;
			    beingModifiedCir = null;
			}
			else{
			    beingModifiedObs = obs;
			    beingModifiedCir = cir;
			}
		    }
		});
		arena.getChildren().add(obstacle);
	    }
	}
	
	/**
	 * Parses paths created from simulation into graphics and displays them
	 */
	private void displayPaths(){
	    if(simulationRun){
		for(Group g : paths){
		    g.setVisible(false);
		}
	    }
	    Path[] path = simulator.getPaths();
	    paths = new Group[3];
	    Color[] colors = {Color.PURPLE, Color.BLUE, Color.GOLD};
	    for(int i = 0; i < paths.length; i++){
		paths[i] = new Group();
		if(path[i] == null){
		    markFailed(i);
		}
		else{
		    for(int pos = 0; pos < path[i].length(); pos++){
			Position point = path[i].getPoint(pos);
			Circle circle = new Circle(getDisplayX(point.getX()), getDisplayY(point.getY()), 5);
			circle.setFill(colors[i]);
			paths[i].getChildren().add(circle);
			if(pos != path[i].length() - 1){
			    Position next = path[i].getPoint(pos + 1);
			    Line line = new Line(getDisplayX(point.getX()), getDisplayY(point.getY()), getDisplayX(next.getX()), getDisplayY(next.getY()));
			    line.setFill(colors[i]);
			    paths[i].getChildren().add(line);
			}
		    }
		}
	    }
	    arena.getChildren().addAll(paths);
	}
	
	/**
	 * Sets up result displays
	 */
	private void setUpResultDisplay(){
	    result = new TextArea[3];
	    for(int i = 0; i < result.length; i++){
		result[i] = new TextArea();
		result[i].setEditable(false);
		result[i].setPrefSize(300, Position.ARENA_HEIGHT() * 50);
	    }
	    result[0].appendText("MidLine Algorithm:\n");
	    result[1].appendText("ModifiedAStar Algorithm:\n");
	    result[2].appendText("GridAStar Algorithm: \n");
	}
	
	/**
	 * Helper method to mark the algorithm that failed
	 * @param failed int index representing the algorithm that failed
	 */
	private void markFailed(int failed){
	    result[failed].appendText("The algorithm failed to create a path\n");
	}
	
	/**
	 * Updates the result screen using paths created by simulation
	 */
	private void updateResult(){ //Currently this is rough estimate
	    int track = 0;
	    for(Path path: simulator.getPaths()){
		if(path == null){
		    track++;
		    continue;
		}
		Position previous = null;
		float dist = 0.0F, angle = 0.0F;
		for(Position pos : path){
		    if(previous != null){
			dist += previous.getDistTo(pos);
			angle += Math.abs(previous.getAngle() - pos.getAngle());
		    }
		    previous = pos;
		}
		float timeTook = dist / (PathPlanSimulator.MAX_STRAIGHT_SPEED * 0.3F) + angle / (PathPlanSimulator.MAX_TURNING_SPEED * 0.3F);
		result[track].appendText(String.format("Total Distance : %.2f m\nTotal Angle Turn: %.2f rad\nEstimate Traversal Time: %.2f s\n", dist, angle, timeTook));
		track++;
	    }
	}
	
	/**
	 * Converts Position x-coordinate to the display x-coordinate
	 * meters -> pixels
	 * @param x_pos Position x-coordinate to convert
	 * @return converted display x-coordinate
	 */
	private double getDisplayX(float x_pos){
	    return (double)((x_pos + Position.ARENA_WIDTH() / 2) * 100);
	}
	
	/**
	 * Converts Position y-coordinate to the display y-coordinate
	 * meters -> pixels
	 * @param y_pos Position y-coordinate to convert
	 * @return converted display y-coordinate
	 */
	private double getDisplayY(float y_pos){
	    return (double)(y_pos * 100);
	}
	
	/**
	 * Converts display x-coordinate to Position x-coordinate
	 * pixels -> meters
	 * @param display_x display x-coordinate to convert 
	 * @return converted Position x-coordinate
	 */
	private float getArenaX(double display_x){
	    return (float)(display_x / 100 - Position.ARENA_WIDTH() / 2);
	}
	
	/**
	 * Converts display y-coordinate to Position y-coordinate
	 * pixels -> meters
	 * @param display_y display y-coordinate to convert
	 * @return converted Position y-coordinate
	 */
	private float getArenaY(double display_y){
	    return (float)(display_y / 100);
	}
	
	/**
	 * Wrapper method for launch method to call outside the class by PathPlanSimulator
	 * @param args
	 */
	private static void launchWrap(String[] args){
	    Application.launch(args);
	}
    }

    /////////////////////////////////////////////////////////////GUI ENDS//////////////////////////////////////////////////////
    
    public static void main(String[] args){
	GUI.launchWrap(args);
    }
    
}
