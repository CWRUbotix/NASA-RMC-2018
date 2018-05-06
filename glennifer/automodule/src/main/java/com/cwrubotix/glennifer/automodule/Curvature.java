package com.cwrubotix.glennifer.automodule;

public class Curvature {
    
    public enum Turn{LEFT, RIGHT};
    private final double leftBound;
    private final double rightBound;
    private final double[] firstDeriv = new double[3];
    private final double[] secondDeriv = new double[2];
    
    public Curvature(double leftBound, double rightBound, double[] eqn){
	if(eqn.length != 4)
	    throw new IllegalArgumentException("Invalid polynomial");
	this.leftBound = leftBound;
	this.rightBound = rightBound;
	firstDeriv[0] = 3 * eqn[0];
	firstDeriv[1] = 2 * eqn[1];
	firstDeriv[2] = eqn[2];
	secondDeriv[0] = 2 * firstDeriv[0];
	secondDeriv[1] = firstDeriv[1];
    }
    
    public double getCurvature(double x){
	checkBound(x);
	double numerator = Math.abs(getSecondDeriv(x));
	double denominator = Math.abs(Math.pow(1 + getFirstDeriv(x) * getFirstDeriv(x), 1.5));
	return numerator / denominator;
    }
    
    public Turn getTurn(double x){
	checkBound(x);
	double check = getSecondDeriv(x);
	if(check < 0){
	    return Turn.RIGHT;
	} else{
	    return Turn.LEFT;
	}
    }
    
    private void checkBound(double x){
	if(x < leftBound || x > rightBound)
	    throw new IllegalArgumentException("x out of bounds");
    }
    
    public double getFirstDeriv(double x){
	return firstDeriv[0] * x * x + firstDeriv[1] * x + firstDeriv[2];
    }
    
    public double getSecondDeriv(double x){
	return secondDeriv[0] * x + secondDeriv[1];
    }
}