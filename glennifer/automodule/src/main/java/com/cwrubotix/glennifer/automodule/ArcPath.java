package com.cwrubotix.glennifer.automodule;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.ArrayList;

/**
 * Represents the ArcPath for the robot
 * @author Seohyun Jung
 *
 */
public class ArcPath{
    /** Points on the path in order*/
    private LinkedList<Position> points = new LinkedList<>();
    /** Stores coefficients for 2nd polynomial representing each arc*/
    private HashMap<Integer, double[]> arcEqs = new HashMap<>();
    /** Map between each point to nearest obstacle*/
    private HashMap<Position, Obstacle> nearestObs = new HashMap<>();
    /** List of Obstacles present in the arena*/
    private ArrayList<Obstacle> obstacles = new ArrayList<>(6);
    /** Instance of ModifiedAStar*/
    private ModifiedAStar astar = new ModifiedAStar();
    
    public ArcPath(Position start, Position end){
	points = astar.computePath(start, end).getPath();
    }
    
    protected Path getPath(){
	return new Path(points);
    }
    
    public Map<Integer, double[]> getArcs(){
	return arcEqs;
    }
    
    public double[] getArc(int segment){
	return arcEqs.get(segment);
    }
    
    public void addObstacle(Position currentPos, Obstacle obs) throws DestinationModified{
	if(!obstacles.contains(obs)){
	    obstacles.add(obs);
	    points = astar.computePath(currentPos, obs).getPath();
	    Position newDest = isDestReasonable(new Path(points));
		if(newDest == null){
		    arcPath();
		} else{
		    points = astar.computePath(currentPos, newDest).getPath();
		    arcPath();
		    throw new DestinationModified(newDest.getX(), newDest.getY());
		}
	}
    }
    
    public void newPath(Position start, Position end) throws DestinationModified{
	points = astar.computePath(start, end).getPath();
	Position newDest = isDestReasonable(new Path(points));
	if(newDest == null){
	    arcPath();
	} else{
	    points = astar.computePath(start, newDest).getPath();
	    arcPath();
	    throw new DestinationModified(newDest.getX(), newDest.getY());
	}
    }
    
    @Override
    public String toString(){
	StringBuilder builder = new StringBuilder();
	Position previous = null;
	int progress = 1;
	for(Position pos : points){
	    if(previous == null){
		previous= pos;
	    } else{
		double[] coefficients = arcEqs.get(progress++);
		builder.append(previous.toString() + " -> " + pos.toString() + "\n");
		if(coefficients != null)
		    builder.append(String.format("Eqn: %.4f x^3 + %.4f x^2 + %.4f x + %.4f", coefficients[0], coefficients[1], coefficients[2], coefficients[3]) + "\n");
		previous = pos;
	    }
	}
	return builder.toString();
    }
    
    private void updateNearestObs(){
	for(Position pos : points){
	    Obstacle nearest = null;
	    for(Obstacle o : obstacles){
		if(nearest == null || nearest.getDistTo(pos) > o.getDistTo(pos))
		    nearest = o;
	    }
	    if(nearestObs.containsKey(pos)){
		nearestObs.replace(pos, nearest);
	    } else{
		nearestObs.put(pos, nearest);
	    }
	}
    }
    
    private void arcPath(){
	updateNearestObs();
	Position previous = null;
	int progress = 1;
	for(Position pos : points){
	    if(previous == null){
		previous = pos;
	    }else{
		makeArc(previous, pos, progress++);
	    }
	}
    }
    
    private void makeArc(Position p1, Position p2, int progress){
	float t1 = getTangent(p1);
	float t2 = getTangent(p2);
	double [][] matrix = new double[4][5];
	matrix[0][0] = Math.pow(p1.getX(), 3);
	matrix[0][1] = Math.pow(p1.getX(), 2);
	matrix[0][2] = p1.getX();
	matrix[0][3] = 1;
	matrix[0][4] = p1.getY();
	matrix[1][0] = Math.pow(p2.getX(), 3);
	matrix[1][1] = Math.pow(p2.getX(), 2);
	matrix[1][2] = p2.getX();
	matrix[1][3] = 1;
	matrix[1][4] = p2.getY();
	matrix[2][0] = 3 * Math.pow(p1.getX(), 2);
	matrix[2][1] = 2 * p1.getX();
	matrix[2][2] = 1;
	matrix[2][3] = 0;
	matrix[2][4] = t1;
	matrix[3][0] = 3 * Math.pow(p2.getX(), 2);
	matrix[3][1] = 2 * p2.getX();
	matrix[3][2] = 1;
	matrix[3][3] = 0;
	matrix[3][4] = t2;
	
	double[] coefficient = solveLinearSystem(matrix);
	arcEqs.put(progress, coefficient);
    }
    
    private double[] solveLinearSystem(double[][] matrix){
	for(int row = 0; row < matrix.length; row ++){
	    for(int col = 0; col < matrix[row].length; col ++){
		if(row == col){
		    matrix[row][row] = 1;
		}else{
		    matrix[row][col] /= matrix[row][row];
		}
	    }
	    for(int delR = row + 1; delR < matrix.length; delR++){
		for(int delC = 0; delC < matrix[delR].length; delC++){
		    if(Math.abs(matrix[row][row]) >= 5e-3)
			matrix[delR][delC] -= matrix[row][delC] * (matrix[delR][row] / matrix[row][row]);
		}
	    }
	}
	double[] ans = new double[4];
	ans[0] = matrix[0][4];
	ans[1] = matrix[1][4];
	ans[2] = matrix[2][4];
	ans[3] = matrix[3][4];
	
	return ans;
    }
    
    private float getTangent(Position p){
	Obstacle o = nearestObs.get(p);
	float m = (p.getY() - o.getY())/(p.getX() - o.getX());
	return -1 / m;
    }
    
    private Position isDestReasonable(Path path){
	float minX = 0.0F, maxX = 0.0F;
	
	for(Position pos : path){
	    if(pos.getX() < minX)
		minX = pos.getX();
	    if(pos.getX() > maxX)
		maxX = pos.getX();
	}
	
	if(Math.abs(maxX - path.getPath().getLast().getX()) > Position.ARENA_WIDTH() * 1 / 3){
	    return new Position(maxX, path.getPath().getLast().getY());
	}
	else if(Math.abs(minX - path.getPath().getLast().getX()) > Position.ARENA_WIDTH() * 1 / 2){
	    return new Position(minX, path.getPath().getLast().getY());
	}
	
	return null;
    }
    
    public static ArcPath arcPath(Path path, ArrayList<Obstacle> obstacles){
	ArcPath a = new ArcPath(path.getPath().getFirst(), path.getPath().getLast());
	a.points = path.getPath();
	a.obstacles = obstacles;
	a.arcPath();
	System.out.println(a.toString());
	return a;
    }
    
    public class DestinationModified extends Exception{
	private float new_dest_x;
	private float new_dest_y;
	
	public DestinationModified(float new_dest_x, float new_dest_y){
	    super();
	    this.new_dest_x = new_dest_x;
	    this.new_dest_y = new_dest_y;
	}
	
	public float getX(){
	    return new_dest_x;
	}
	
	public float getY(){
	    return new_dest_y;
	}
    }
    
}