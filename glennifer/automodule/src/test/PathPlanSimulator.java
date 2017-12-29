package test;

import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.MidLine;
import main.java.com.cwrubotix.glennifer.automodule.ModifiedAStar;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;

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
    private Path[] paths = new Path[2]; //TODO Add AStarGrid
    private PathFindingAlgorithm[] finders = new PathFindingAlgorithm[2]; //TODO Add AStarGrid
    private Position initialPos;
    private Position destination;

    
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
	finders[0] = new MidLine();
	finders[1] = new ModifiedAStar();
	for(int i = 0; i < paths.length; i ++){
	    paths[i] = finders[i].computePath(initialPos, destination);
	}
	//TODO ADD AStarGrid
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
	    Obstacle newObstacle = new Obstacle(newObstacleX, newObstacleY, OBSTACLE_SIZE);
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
    

    private void moveRobot(int robot){
	//TODO
    }
    
    public void runSimulation(){
	//TODO
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
	private TextArea[] result = new TextArea[2];
	private Group[] paths = new Group[2];
	
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
	    VBox left = new VBox();
	    left.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    left.getChildren().addAll(result[0], result[1]);
	    VBox right = new VBox();
	    right.setPrefSize(300, Position.ARENA_HEIGHT() * 100);
	    right.getChildren().addAll(result[2], result[3]);
	    pane.setLeft(left);
	    pane.setRight(right);
	    pane.setBottom(startButton);
	    pane.setCenter(arena);
	    primaryStage.setScene(new Scene(pane));
	}
	
	private void setUpArena(){
	    arena.setPrefSize(Position.ARENA_WIDTH() * 100, Position.ARENA_HEIGHT() * 100);
	    arena.setOnMouseClicked(new EventHandler<MouseEvent>(){

		@Override
		public void handle(MouseEvent event) {
		    if(beingModifiedObs == null)
			return;
		    float x = getArenaX(event.getX());
		    float y = getArenaY(event.getY());
		    if(simulator.modifyObstacle(beingModifiedObs, x, y)){
			beingModifiedCir.setCenterX(event.getX());
			beingModifiedCir.setCenterY(event.getY());
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
	    Rectangle start = new Rectangle(getDisplayX(simulator.getInitialPos().getX()), getDisplayY(simulator.getInitialPos().getY()), 75, 50);
	    line1.setFill(Color.LIGHTGRAY);
	    line2.setFill(Color.LIGHTGRAY);
	    line3.setFill(Color.LIGHTGOLDENRODYELLOW);
	    start.setFill(Color.RED);
	    bin.setFill(Color.BLUE);
	    arena.getChildren().addAll(line1, line2, line3, bin, start);
	    setUpObstacles();
	}
	
	private void setUpObstacles(){
	    for(Obstacle p : simulator.getObstacles()){
		Circle obstacle = new Circle(getDisplayX(p.getX()), getDisplayY(p.getY()), 15, Color.DARKGREY);
		obstacle.setOnMouseClicked(new EventHandler<MouseEvent>(){
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
	    paths = new Group[2]; // TODO Change as more algorithms are added.
	    Color[] colors = {Color.PURPLE, Color.AQUA, Color.GOLD};
	    for(int i = 0; i < paths.length; i++){
		if(paths[i] == null)
		    continue;
		paths[i] = new Group();
		for(int pos = 0; pos < path[i].length(); i ++){
		    Position point = path[i].getPoint(pos);
		    Circle circle = new Circle(getDisplayX(point.getX()), getDisplayY(point.getY()), 2.5);
		    circle.setFill(colors[i]);
		    paths[i].getChildren().add(circle);
		    if(pos != path[i].length() - 1){
			Position next = path[i].getPoint(pos + 1);
			Line line = new Line(getDisplayX(point.getX()), getDisplayY(point.getY()), getDisplayX(next.getX()), getDisplayY(next.getY()));
			line.setFill(Color.BLACK);
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
	
	private void updateResult(){ //Currently this is rough estimate
	    int track = 0;
	    for(Path path: simulator.getPaths()){
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
		result[track].appendText("Total Distance : "+ dist +"\nTotal Angle Turn: "+ angle +"\nEstimate Traversal Time: " + timeTook);
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
