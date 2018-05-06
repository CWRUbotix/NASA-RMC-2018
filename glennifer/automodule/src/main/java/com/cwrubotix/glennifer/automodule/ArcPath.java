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
    /** Stores coefficients for 3rd polynomial representing each arc*/
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
    
    public void addObstacle(Position currentPos, Obstacle obs){
	if(!obstacles.contains(obs)){
	    obstacles.add(obs);
	    points = astar.computePath(currentPos, obs).getPath();
	    arcPath();
	}
    }
    
    public void newPath(Position start, Position end){
	points = astar.computePath(start, end).getPath();
	arcPath();
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
		    builder.append(String.format("Eqn: %.5f x^3 + %.5f x^2 + %.5f x + %.5f", coefficients[0], coefficients[1], coefficients[2], coefficients[3]) + "\n");
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
	double t1, t2;
	if(progress == 1){
	    if(p1.getHeading() == Math.PI / 2 || p1.getHeading() == Math.PI * 3 / 2)
		t1 = 0;
	    else
		t1 = -Math.tan(p1.getHeading() + Math.PI / 2);
	}else{
	    t1 = getTangent(p1);
	}
	t2 = getTangent(p2);
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
	
	double[] coefficient = solveLinearSystem(matrix, p1, p2);
	arcEqs.put(progress, coefficient);
    }
    
    private double[] solveLinearSystem(double[][] matrix, Position p1, Position p2){
	double[][] store = new double[matrix.length][matrix[0].length];
	
	for(int row = 0; row < matrix.length; row++){
	    for(int col = 0; col < matrix[row].length; col++){
		store[row][col] = matrix[row][col];
	    }
	}
	
	for(int row = 0; row < matrix.length; row ++){
	    if(store[row][row] == 0.0){
		int swap = -1;
		for(int r = row + 1; r < matrix.length && swap == -1; r++){
		    if(store[r][row] != 0.0)
			swap = r;
		}
		store = R3(store, row, swap);
	    }
	    
	    store[row] = R2(store[row], row);
	    
	    for(int r = 0; r< matrix.length; r++){
		if(r != row){
		    store[r] = R1(store[row], store[r], store[r][row], row);
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
    
    private double getTangent(Position p){
	Obstacle o = nearestObs.get(p);
	double m;
	if(o == null){
	    if(p.getHeading() == Math.PI / 2 || p.getHeading() == Math.PI * 3 / 2)
		m = 0;
	    else
		m = -Math.tan(p.getHeading() + Math.PI / 2);
	} else{
	    m = (p.getY() - o.getY())/(p.getX() - o.getX());

	}
	return -1 / m;
    }
    
    /**
     * Applies R1 row operation on r2 by r1 with factor given
     * with purpose to make row-th entry 0.
     * 
     * @param r1
     * @param r2
     * @return r2 after R1 operation performed.
     */
    private double[] R1(double[] r1, double[] r2, double factor, int row){
	double[] result = new double[r2.length];
	for(int i = 0; i < result.length; i ++){
	    if(i == row){
		result[i] = 0;
	    }else{
		result[i] = r2[i] - (r1[i] * factor);
	    }
	}	
	return result;
    }
    
    /**
     * Applies R2 on r1 with purpose to make row-th entry 1
     * 
     * @param r1
     * @param row
     * @return r1 after R2 operation performed
     */
    private double[] R2(double[] r1, int row){
	/*Copying array in*/
	double[] result = new double[r1.length];
	for(int i = 0; i < result.length; i ++){
	    result[i] = r1[i];
	}
	
	double factor = result[row];
	for(int i = 0; i < result.length; i++){
	    if(i == row){
		result[i] = 1;
	    } else{
		result[i] = result[i] / factor;
	    }
	}
	
	return result;
    }
    
    private double[][] R3(double[][] matrix, int r1, int r2){
	/*Copying the matrix in*/
	double[][] result = new double[matrix.length][matrix[r1].length];
	for(int row = 0; row < matrix.length; row++){
	    for(int col = 0; col < matrix[row].length; col ++){
		result[row][col] = matrix[row][col];
	    }
	}
	
	/*Switching rows*/
	double[] save = matrix[r1];
	result[r1] = result[r2];
	result[r2] = save;
	
	return result;
    }
        
    public static ArcPath arcPath(Path path, ArrayList<Obstacle> obstacles){
	ArcPath a = new ArcPath(path.getPath().getFirst(), path.getPath().getLast());
	a.points = path.getPath();
	a.obstacles = obstacles;
	a.arcPath();
	return a;
    }
        
}
