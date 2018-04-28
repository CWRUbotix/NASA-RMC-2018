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
    private HashMap<Integer, float[]> arcEqs = new HashMap<>();
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
    
    public Map<Integer, float[]> getArcs(){
	return arcEqs;
    }
    
    public float[] getArc(int segment){
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
		float[] coefficients = arcEqs.get(progress++);
		builder.append(previous.toString() + " -> " + pos.toString() + "\n");
		builder.append(String.format("%.4f", coefficients[0]) + "x^2 + " 
		+ String.format("%.4f", coefficients[1]) + "x + " 
			+ String.format("%.4f", coefficients[2]) + "\n");
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
	float m,t1,t2,x3,y3,a,b,c;
	m = (p1.getY() - p2.getY()) / (p1.getX() - p2.getX());
	t1 = getTangent(p1);
	t2 = getTangent(p2);
	x3 = (p1.getX() + p2.getX()) / 2;
	y3 = (float) (p1.getY() 
		+ t1 * (x3 - p1.getX()) 
		+ (3 * m - 2 * t1 - t2) / (p2.getX() - p1.getX()) * Math.pow(x3 - p1.getX(), 2) 
		+ (t1 + t2 - 2 * m) / Math.pow(p2.getX() - p1.getX(), 2) * Math.pow(x3 - p1.getX(), 3));
	a = ((p1.getY() - y3) / (p1.getX() - x3) - (p1.getY() - p2.getY()) / (p1.getX() - p2.getX())) / (x3 - p2.getX());
	b = (p1.getY() - p2.getY())/(p1.getX() - p2.getX()) - a * (p1.getX() + p2.getX());
	c = p1.getY() - a * p1.getX() * p1.getX() - b * p1.getX();
	
	float[] coefficient = {a, b, c};
	arcEqs.put(progress, coefficient);
    }
    
    private float getTangent(Position p){
	Obstacle o = nearestObs.get(p);
	float m = (p.getY() - o.getY())/(p.getX() - o.getX());
	return -1 / m;
    }
    
}