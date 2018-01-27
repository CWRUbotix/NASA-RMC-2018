package test;

import org.junit.*;

import static org.junit.Assert.*;

import main.java.com.cwrubotix.glennifer.automodule.MidLine;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.Position;

/*Simple sanity check whether the algorithm works in general case*/
public class MidLineJUnit {

    @Test
    public void test() {
        Position start = new Position((float) (-1.89 / 2), (float) (1.5 / 2));
        Position end = new Position((float) (1.89 / 2), (float) (1.5 + 2.94 + 0.75));
        MidLine pathFinder = new MidLine();

        verifyPath(pathFinder.computePath(start, end), start, end);

        Obstacle o1 = new Obstacle(-0.75F, 1.0F, 0.15F);
        Obstacle o2 = new Obstacle(0.0F, 3.0F, 0.15F);
        Position cp1 = new Position(-0.5F, 1.5F, 0.0, 0.0F);
        Position cp2 = new Position(-0.3F, 2.0F, 0.0, 0.0F);

        verifyPathWithCP(pathFinder.computePath(cp1, o1), cp1, start, end);
        verifyPathWithCP(pathFinder.computePath(cp2, o2), cp2, start, end);
    }

    private void verifyPath(Path path, Position start, Position end) {
        assertEquals("path created failed to have valid start point", start, path.getPoint(0));
        assertEquals("path created failed to have valid end point", end, path.getPoint(path.length() - 1));
    }

    private void verifyPathWithCP(Path path, Position currentPos, Position start, Position end) {
        boolean contains = false;
        for (Position p : path) {
            if (currentPos.equals(p))
                contains = true;
        }
        if (!contains) fail("Returned path does not contain current position of the robot");

        verifyPath(path, start, end);
    }
}