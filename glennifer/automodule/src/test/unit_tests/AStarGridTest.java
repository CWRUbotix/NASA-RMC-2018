package unit_tests;

import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.astargrid.AStarGrid;
import main.java.com.cwrubotix.glennifer.automodule.astargrid.FuzzyArenaGraph;
import org.junit.Assert;
import org.junit.Test;

import java.util.LinkedList;


public class AStarGridTest {
    @Test
    public void testSimpleGrid() throws Exception {
        FuzzyArenaGraph grid = new FuzzyArenaGraph(2.5, 1.5, 0.25);
        AStarGrid aStarGrid = new AStarGrid(grid);

        LinkedList<Position> solutionList = new LinkedList<>();
        solutionList.add(new Position(0, 0, 0, 0));
        solutionList.add(new Position(0, 0.5F, 0, 0));
        solutionList.add(new Position(0, 1.0F, 0, 0));
        Path expected = new Path(solutionList);

        Path actual = aStarGrid.computePath(new Position(0, 0, 0, 0),
                new Position(0, 1.0F, 0, 0));

        Assert.assertEquals(expected, actual);
    }

    @Test
    public void testArenaGrid() throws Exception {
        AStarGrid aStarGrid = new AStarGrid(0.1);

        LinkedList<Position> solutionList = new LinkedList<>();
        float pos = 0.0F;
        while (pos <= 1.0F) {
            solutionList.add(new Position(0, pos, 0, 0));
            pos += 0.2;
        }
        Path expected = new Path(solutionList);

        Path actual = aStarGrid.computePath(new Position(0, 0, 0, 0),
                new Position(0, 1.0F, 0, 0));

        Assert.assertEquals(expected, actual);
    }
}