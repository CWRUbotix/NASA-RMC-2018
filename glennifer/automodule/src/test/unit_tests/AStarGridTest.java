package unit_tests;

import com.cwrubotix.glennifer.automodule.Path;
import com.cwrubotix.glennifer.automodule.Position;
import com.cwrubotix.glennifer.automodule.astargrid.AStarGrid;
import com.cwrubotix.glennifer.automodule.astargrid.FuzzyArenaGraph;
import org.junit.Assert;
import org.junit.Test;

import java.util.LinkedList;

import static org.junit.Assert.*;

public class AStarGridTest {
    @Test
    public void testSimpleGrid() throws Exception {
        FuzzyArenaGraph grid = new FuzzyArenaGraph(2.5, 1.5, 0.5);
        AStarGrid aStarGrid = new AStarGrid(grid);

        LinkedList<Position> solutionList = new LinkedList<>();
        solutionList.add(new Position(0, 0, 0, 0));
        solutionList.add(new Position(0, 1.0F, 0, 0));
        Path expected = new Path(solutionList);

        Path actual = aStarGrid.computePath(new Position(0, 0, 0, 0),
                new Position(0, 1.0F, 0, 0));

        Assert.assertEquals(expected, actual);
    }
}