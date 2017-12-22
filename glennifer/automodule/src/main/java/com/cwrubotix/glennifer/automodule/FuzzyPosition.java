package main.java.com.cwrubotix.glennifer.automodule;

public class FuzzyPosition extends Position {
    private double error;

    public FuzzyPosition(float xPos, float yPos, double error) {
        super(xPos, yPos, 0.0, 0.0F);
        this.error = error;
    }

    public double getError() {
        return error;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof FuzzyPosition) {
            FuzzyPosition compare = (FuzzyPosition) obj;
            double error = (this.error < compare.error) ? this.error : compare.error;
            return Math.abs(this.getX() - compare.getX()) <= error && Math.abs(this.getY() - compare.getY()) <= error;
        }
        else if (obj instanceof Position) {
            Position compare = (Position) obj;
            return Math.abs(this.getX() - compare.getX()) <= error && Math.abs(this.getY() - compare.getY()) <= error;
        }
        return false;
    }

    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ", +/- " + error + ")";
    }

    @Override
    public int hashCode() {
        return super.hashCode() | Float.floatToRawIntBits((float) error);
    }
}
