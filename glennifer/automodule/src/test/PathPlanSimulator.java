package test;

import main.java.com.cwrubotix.glennifer.automodule.MidLine;
import main.java.com.cwrubotix.glennifer.automodule.ModifiedAStar;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;
import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.astargrid.AStarPathFinder;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.PathFinder;

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
    private PathFinder<?>[] finders = new PathFinder[3];
    private Position initialPos;
    private Position destination;
    private String errorMessages = "";
    
    /*Constants:*/
    /** Stores Max straight speed of the robot. Unit: m/s*/
    private static final float MAX_STRAIGHT_SPEED = 3.32F;
    /** Stores Max turning speed of the robot. Unit: rad/s*/
    private static final float MAX_TURNING_SPEED = (float)Math.PI;
    /** Stores robot's width in meters*/
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
	    finders[2] = new AStarPathFinder(initialPos, destination, 5e-2);//TODO Add AStarGrid
	    for(int i = 0; i < paths.length; i ++){
	      if(finders[i] == null)
		      continue;
	      paths[i] = finders[i].getPath();
	    }
    }
    
    public Position getInitialPos(){
	    return initialPos;
    }
    
    public Position getDestination(){
	    return destination;
    }
    
    public Obstacle[] getObstacles(){
	    return obstacles;
    }
    
    
    public Path[] getPaths(){
	    return paths;
    }
    
    public String getErrorMessages(){
	    return errorMessages;
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
    

    private void moveRobot(int robot){
	boolean arrived = false;
	PathFinder<?> finder = finders[robot];
	Path path = finder.getPath();
	Position currentPos = path.getPoint(0);
	int progress = 0;
	ArrayList<Obstacle> obstacles = new ArrayList<>(6);
	for(int i = 0; i < 6; i++){
	    obstacles.add(getObstacles()[i]);
	}
	while(!arrived){
	    System.out.println("Sorting started");
	    obstacles.sort(Position.getComparatorByDistTo(currentPos));
	    System.out.println("Sorting Ended");
	    Obstacle encountered = null;
	    for(Obstacle obs : obstacles){
		Position temp = getEncounter(path, progress, obs);
		if(temp != null){
		    currentPos = temp;
		    finder.setCurrentPos(currentPos);
		    finder.registerObstacle(obs);
		    path = finder.getPath();
		    encountered = obs;
		    System.out.println("Path modified");
		    break;
		}
	    }
	    if(encountered != null){
		obstacles.remove(encountered);
		progress = path.getPath().indexOf(currentPos);
	    }
	    else{
		arrived = true;
	    }
	}
	System.out.println("Robot arrived");
	paths[robot] = path;
    }
    
    private Position getEncounter(Path path, int progress, Obstacle obs){
	Position p1 = path.getPoint(progress);
	Position p2 = path.getPoint(progress + 1);
	float x1 = p1.getX();
	float y1 = p1.getY();
	float x2 = p2.getX();
	float y2 = p2.getY();
	float cx = obs.getX();
	float cy = obs.getY();
		
	double slope = (y2 - y1)/(x2 - x1);
	double y_intercept = y1 - slope * x1;
	
	double a = 1 + Math.pow(slope, 2);
	double b = 2 * slope * (y_intercept - cy) - 2 * cx;
	double c = (Math.pow(cx, 2) + Math.pow((y_intercept - cy), 2)) - Math.pow(KINECT_RANGE, 2) / 4;
	
	double check = Math.pow(b, 2) - 4 * a * c;

	if(check < 0){
	    System.out.println("Didn't find obstacle");
	    return null;
	}
	System.out.println("Found Obstacle");
	float rx1 = (float)((-b + Math.sqrt(check)) / (2 * a));
	float ry1 = (float)(slope * rx1 + y_intercept);
	float rx2 = (float)((-b - Math.sqrt(check)) / (2 * a));
	float ry2 = (float)(slope * rx2 + y_intercept);
	
	Position r1 = new Position(rx1, ry1);
	Position r2 = new Position(rx2, ry2);
	
	if(p1.getDistTo(r1) < p1.getDistTo(r2))
	    return r1;
	else
	    return r2;
    }
    
    public void runSimulation(){
	for(int i = 0; i < finders.length; i++){
	    try{
		moveRobot(i);
	    }
	    catch(RuntimeException e){ //TODO Switch to AlgorithmFailure Exception
		String failedOne = "";
		switch(i){
		case 0: failedOne = "MidLine"; break;
		case 1: failedOne = "ModifiedAStar"; break;
		case 2: failedOne = "AStarGrid"; break;
		}
		errorMessages = errorMessages + failedOne + ": \n" + e.getMessage() + "\n\n";
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
	
	private PathPlanSimulator simulator;
	private Stage primaryStage;
	private BorderPane pane = new BorderPane();
	private Pane arena = new Pane();
	private boolean modificationAllowed = true;
	private boolean simulationRun = false;
	private Obstacle beingModifiedObs = null;
	private Circle beingModifiedCir = null;
	private TextArea[] result = new TextArea[3];
	private TextArea errorMessages = new TextArea();
	private Group[] paths = new Group[3];
	
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
	    Position start = new Position(0.0F, 0.0F, Math.PI / 2, 0.0F);
	    Position destination = new Position(0.0F, 0.0F);
	    if(!start.setX(Float.parseFloat(args[0])) || !start.setY(Float.parseFloat(args[1]))
		    || !destination.setX(Float.parseFloat(args[2])) || !destination.setY(Float.parseFloat(args[3])))
		throw new RuntimeException("Given coordinates are invalid to set up simulation");
	    simulator = new PathPlanSimulator(start, destination);
	}
	
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
		    simulationRun = true;
		    simulator.runSimulation();
		    displayPaths();
		    updateResult();
		}
	    });
	    
	    errorMessages.setEditable(false);
	    errorMessages.setPrefSize(300, Position.ARENA_HEIGHT() * 50);
	    VBox left = new VBox();
	    left.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    left.getChildren().addAll(result[0], result[1]);
	    VBox right = new VBox();
	    right.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    right.getChildren().addAll(result[2], errorMessages, startButton);
	    left.setAlignment(Pos.CENTER);
	    right.setAlignment(Pos.CENTER);
	    pane.setLeft(left);
	    pane.setRight(right);
	    pane.setCenter(arena);
	    primaryStage.setScene(new Scene(pane));
	}
	
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
	
	private void displayPaths(){
	    Path[] path = simulator.getPaths();
	    paths = new Group[3];
	    Color[] colors = {Color.PURPLE, Color.AQUA, Color.GOLD};
	    for(int i = 0; i < paths.length; i++){
		paths[i] = new Group();
		if(path[i] == null){
		    errorMessages.appendText(simulator.getErrorMessages());
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
	
	private void markFailed(int failed){
	    result[failed].appendText("The algorithm failed to create a path");
	}
	
	private void updateResult(){ //Currently this is rough estimate
	    int track = 0;
	    for(Path path: simulator.getPaths()){
		if(path == null)
		    continue;
		Position previous = null;
		float dist = 0.0F, angle = 0.0F;
		for(Position pos : path){
		    if(previous != null){
			dist += previous.getDistTo(pos);
			angle += previous.getAngleTurnTo(pos);
		    }
		    previous = pos;
		}
		float timeTook = dist / (PathPlanSimulator.MAX_STRAIGHT_SPEED * 0.3F) + angle / (PathPlanSimulator.MAX_TURNING_SPEED * 0.3F);
		result[track].appendText("Total Distance : "+ dist +"\nTotal Angle Turn: "+ angle +"\nEstimate Traversal Time: " + timeTook + "s");
		track++;
	    }
	}
	
	private double getDisplayX(float x_pos){
	    return (double)((x_pos + Position.ARENA_WIDTH() / 2) * 100);
	}
	
	private double getDisplayY(float y_pos){
	    return (double)(y_pos * 100);
	}
	
	private float getArenaX(double display_x){
	    return (float)(display_x / 100 - Position.ARENA_WIDTH() / 2);
	}
	
	private float getArenaY(double display_y){
	    return (float)(display_y / 100);
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
