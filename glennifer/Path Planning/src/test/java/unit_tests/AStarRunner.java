package unit_tests;

import java.applet.Applet;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;
import java.util.Queue;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import algorithms.AStarPolygonal;
import commands.MidLevelCommand;

public class AStarRunner extends Applet implements MouseListener{

	int xPos;
	int yPos;
	boolean first_time = true;
	ArrayList<ArrayList<int[]>> obstacles = new ArrayList<ArrayList<int[]>>();
	int[] start = new int[2];
	int[] end = new int[2];
	int mode = 0;
	static final int IDLE_MODE = 0;
	static final int END_MODE = 3;
	static final int START_MODE = 2;
	static final int OBS_MODE = 1;

	ArrayList<int[]> temp_obstacle = new ArrayList<int[]>();
	
	ArrayList<int[]> travel_path = new ArrayList<int[]>();

	public void init() {
		setSize(400, 400);
		setBackground(Color.white);
		setForeground(Color.black);
		addMouseListener(this);

		KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher() {

			@Override
			public boolean dispatchKeyEvent(KeyEvent ke) {
				synchronized (AStarRunner.class) {
					switch (ke.getID()) {
					case KeyEvent.KEY_PRESSED:

						int keyCode = ke.getKeyCode();
						switch (keyCode) {
						case KeyEvent.VK_O:
							mode = AStarRunner.OBS_MODE;
							break;
						case KeyEvent.VK_S:
							mode = AStarRunner.START_MODE;
							break;
						case KeyEvent.VK_E:
							mode = AStarRunner.END_MODE;
							break;
						}

						break;

					case KeyEvent.KEY_RELEASED:
						if (mode == AStarRunner.OBS_MODE) {
							ArrayList<int[]> adder = new ArrayList<int[]>();
							for (int[] a : temp_obstacle){
								adder.add(a);
							}
							obstacles.add(adder);
							temp_obstacle.clear();
							//repaint();
						}
						mode = AStarRunner.IDLE_MODE;
						break;
					}
					return false;
				}
			}
		});
	}

	public void paint(Graphics g) {

		g.setColor(Color.GRAY);
		g.drawLine(200, 0, 200, 400);
		g.drawLine(0, 200, 400, 200);
		
		g.setColor(Color.BLACK);
		if (!(obstacles.isEmpty())) {
			//System.out.println(obstacles.size() + ": obstacles");
			for (ArrayList<int[]> obs : obstacles) {
				//System.out.println(obs.size() + ": obs indv");
				drawPolygon(g, obs);
			}			
		}
		
		g.setColor(Color.ORANGE);
		if (!(temp_obstacle.isEmpty())){
			//System.out.println(temp_obstacle.size() + ": temp");
			drawPolygon(g, temp_obstacle);
		}

		g.setColor(Color.GREEN);
		g.fillRect(start[0], start[1], 4, 4);
		g.setColor(Color.RED);
		g.fillRect(end[0], end[1], 4, 4);
		
		//System.out.println(obstacles.size() + ": obstacles");
		travel_path = AStarPolygonal.planPathPoints(0, start[0], start[1], end[0], end[1], obstacles);
		System.out.println("Commands:");
		Queue<MidLevelCommand> commands = AStarPolygonal.planPath(0, start[0], start[1], end[0], end[1], obstacles);
		for (MidLevelCommand a : commands)
			System.out.println(a);
		
		g.setColor(Color.YELLOW);
		if (!(travel_path.isEmpty())){
			for (int i = 0; i < travel_path.size() - 1; i++){
				//System.out.println("TP:\n1) " + travel_path.get(i)[0] + ", " + travel_path.get(i)[1] + "\n2) " + travel_path.get(i + 1)[0] + ", " + travel_path.get(i + 1)[1]);
				g.drawLine(travel_path.get(i)[0], travel_path.get(i)[1], travel_path.get(i + 1)[0], travel_path.get(i+1)[1]);
			}
		}
	}

	@Override
	public void mouseClicked(MouseEvent arg0) {
		xPos = arg0.getX();
		yPos = arg0.getY();
		switch (mode) {
		case AStarRunner.OBS_MODE:
			temp_obstacle.add(new int[] { xPos, yPos });
			break;
		case AStarRunner.START_MODE:
			start[0] = xPos;
			start[1] = yPos;
			break;
		case AStarRunner.END_MODE:
			end[0] = xPos;
			end[1] = yPos;
			break;
		}
		repaint();
	}

	@Override
	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mousePressed(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseReleased(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	private void drawPolygon(Graphics g, ArrayList<int[]> polygon){
		for (int i = 0; i < polygon.size() - 1; i++) {
			g.drawLine(polygon.get(i)[0], polygon.get(i)[1], polygon.get(i + 1)[0],
					polygon.get(i + 1)[1]);
		}
		g.drawLine(polygon.get(0)[0], polygon.get(0)[1], polygon.get(polygon.size() - 1)[0],
				polygon.get(polygon.size() - 1)[1]);
	}
}
