package main.java.com.cwrubotix.glennifer.automodule;

public class Obstacle extends Position {

    private float radius;

    public Obstacle(float x_pos, float y_pos, float radius) {
        super(x_pos, y_pos);
        this.radius = radius;
    }

    public float getRadius() {
        return radius;
    }

    public Obstacle(Position obs) {
        this(obs.getX(), obs.getY(), 0.15F);
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }
}