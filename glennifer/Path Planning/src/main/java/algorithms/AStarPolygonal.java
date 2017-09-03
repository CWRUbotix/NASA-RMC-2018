package algorithms;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

import commands.MidLevelCommand;

public class AStarPolygonal {

	/**
	 * The robot's speed in meters per second
	 */
	public static final double lateral_speed = 0.86;

	/**
	 * Robot's rate of turning in radians per second
	 */
	public static final double turn_speed = 90 * Math.PI / 180 / 6;

	public static class Point {
		int x;
		int y;

		public Point(int x, int y) {
			this.x = x;
			this.y = y;
		}

		boolean checkLoc(Point p) {
			if (p.x == this.x && p.y == this.y)
				return true;
			else
				return false;
		}
	}

	static class LineSegment {
		Node p;
		Node q;
		// double length;
		// double angle_change;
		// double heuristic, cost;

		LineSegment(Node p, Node q) {
			this.p = p;
			this.q = q;

			// length = Math.sqrt(Math.pow(p.x - q.x, 2) + Math.pow(p.y - q.y,
			// 2));
		}

		/**
		 * Determines if given point is an endpoint
		 * 
		 * @param a
		 *            The point to be checked
		 * @return (0) if not an endpoint, (1) if it is p, (2) if it is q
		 */
		int findPoint(Point a) {
			if (a.x == p.x && a.y == p.y)
				return 1;
			else if (a.x == q.x && a.y == q.y)
				return 2;
			else
				return 0;
		}
	}

	/**
	 * Nodes are points with an array of line segments to travel along
	 * 
	 * @author Shota
	 *
	 */
	static class Node extends Point {
		// ArrayList<Node> paths;
		Node parent;
		double heuristic, cost;
		double parent_angle_change = 0, parent_distance = 0;

		Node(int x, int y) {
			super(x, y);
		}

		Node(Point p) {
			super(p.x, p.y);
		}

		public String toString() {
			return "(" + x + "," + y + ")";
		}
	}

	static class Edge extends LineSegment {
		boolean above;

		Edge(Node p, Node q) {
			super(p, q);
		}

	}

	static class Polygon {
		Point[] vertices;
		int minX, maxX;
		int minY, maxY;
		int boxMinX, boxMaxX;
		int boxMinY, boxMaxY;
		Edge[] edges;

		Polygon(Point[] vertices) {
			this.vertices = vertices;
			minX = this.vertices[0].x;
			minY = this.vertices[0].y;
			// Sets the "box" the obstacle is in
			for (Point a : this.vertices) {
				minX = Math.min(minX, a.x);
				maxX = Math.max(maxX, a.x);
				minY = Math.min(minY, a.y);
				maxY = Math.max(maxY, a.y);
			}

			boxMinX = minX - 10;
			boxMaxX = maxX + 10;
			boxMinY = minY - 10;
			boxMaxY = maxY + 10;
		}

		boolean checkInsideEst(Point p) {
			if (p.x > minX && p.x < maxX && p.y > minY && p.y < maxY)
				return true;
			else
				return false;
		}

		Point[] boxPoints() {
			return new Point[] { new Point(boxMinX, boxMinY), new Point(boxMaxX, boxMinY), new Point(boxMaxX, boxMaxY),
					new Point(boxMinX, boxMaxY) };
		}

		/**
		 * ---------------Important-------------- This method is unfinished. Try
		 * not to use.
		 * 
		 * @param p
		 * @return
		 */
		boolean checkInsidePrecise(Point p) {
			return false;
		}
	}

	private Point start;

	private Point end;

	public static Queue<MidLevelCommand> planPath(double initial_angle, int startX, int startY, int endX, int endY,
			ArrayList<ArrayList<int[]>> obstacle_points) {
		// Creates points for the start and end coordinates
		Point start = new Point(startX, startY);
		Point end = new Point(endX, endY);

		// Checkes if there are any obstacles
		ArrayList<Node> travel_nodes = new ArrayList<Node>();
		// if (obstacles.isEmpty()) {
		// travel_nodes.add(new Node(start));
		// Node end_node = new Node(end);
		// end_node.cost = Math.sqrt(Math.pow(start.x - end.x, 2) +
		// Math.pow(start.y - end.y, 2))
		// / AStarPolygonal.lateral_speed;
		// travel_nodes.add(new Node(end_node));
		// // return travel_points;
		// } else {
		// Formats the obstacle array list into an arraylist of Points

		// Runs the AStar Algorithm
		travel_nodes = AStar(initial_angle, start, end, createObstacles(obstacle_points));
		// }

		Queue<MidLevelCommand> commands = new LinkedList<MidLevelCommand>();
		if (travel_nodes.get(0).checkLoc(start))
			travel_nodes.remove(0);
		else
			System.err.println("Something has gone terribly wrong. Start is not the first node");

		for (int i = 0; i < travel_nodes.size(); i++) {
			commands.add(new MidLevelCommand(
					"Turn " + travel_nodes.get(i).parent_angle_change * 180 / Math.PI + " degrees"));
			commands.add(new MidLevelCommand("Travel " + travel_nodes.get(i).parent_distance + " meters"));
		}

		return commands;

	}

	/**
	 * This Method is to be used with the applet tester
	 * 
	 * Enter a starting Point and an ending Point, as well as an array of
	 * obstacles to avoid. The obstacles will be represented as Point arrays,
	 * with connected vertices listed consecutively.
	 * 
	 * @param initial_angle
	 *            The starting angle in degrees measured from standard position
	 *            (initially parallel to +x axis)
	 * @param start
	 *            The starting Point
	 * @param end
	 *            The ending Point
	 * @param obstacle_points
	 *            An array of Point arrays. Each Point array represents a
	 *            polygon that serves as an obstacle.
	 * @return A list of points that make up the shortest path from the starting
	 *         point to the end.
	 */
	public static ArrayList<int[]> planPathPoints(double initial_angle, int startX, int startY, int endX, int endY,
			ArrayList<ArrayList<int[]>> obstacle_points) {

		// Creates points for the start and end coordinates
		Point start = new Point(startX, startY);
		Point end = new Point(endX, endY);

		// Checkes if there are any obstacles
		ArrayList<int[]> travel_points = new ArrayList<int[]>();
		// if (obstacles.isEmpty()) {
		// travel_points.add(new int[] { startX, startY });
		// travel_points.add(new int[] { endX, endY });
		// return travel_points;
		// }

		// Formats the obstacle array list into an arraylist of Points

		// Runs the AStar Algorithm
		ArrayList<Node> travel_nodes = AStar(initial_angle, start, end, createObstacles(obstacle_points));

		for (Node node : travel_nodes) {
			travel_points.add(new int[] { node.x, node.y });
		}
		return travel_points;
	}

	/**
	 * Applies a heuristic cost to all of the given Nodes, and adds the start
	 * and end nodes.
	 * 
	 * @param initial_angle
	 *            in radians for now
	 * @param start
	 * @param end
	 * @param obstacle_points
	 * @return The list of Nodes with heuristic costs. Null if start or end
	 *         point isn't valid.
	 */
	private static ArrayList<Node> AStar(double initial_angle, Point start_point, Point end_point,
			ArrayList<Polygon> obstacles) {

		// Create start and end nodes with empty path fields
		Node start = new Node(start_point.x, start_point.y);
		start.cost = 0;
		Node end = new Node(end_point.x, end_point.y);

		// Add start and end nodes to list, create and add nodes for all
		// obstacle box points and the end point
		ArrayList<Node> open_nodes = new ArrayList<Node>();
		open_nodes.add(end);
		
		if (obstacles.isEmpty()){
			open_nodes.add(0, start);
			return open_nodes;
		}
		
		for (Polygon obs : obstacles) {
			Point[] temp = obs.boxPoints();
			for (Point a : temp){
				open_nodes.add(new Node(a));
			}
		}
		// ArrayList<Polygon> obstacles = new ArrayList<Polygon>();

		// Adds all of the obstacle's Box points to open_nodes

		if (!checkValid(start, obstacles) && !checkValid(end, obstacles))
			return null;

		// Creates the priority queue that orders Nodes by lowest cost first
		PriorityQueue<Node> heuristic_nodes;
		heuristic_nodes = new PriorityQueue<Node>(open_nodes.size(), new Comparator<Node>() {
			@Override
			public int compare(Node n1, Node n2) {
				return n1.cost < n2.cost ? -1 : n1.cost > n2.cost ? 1 : 0;
			}
		});
		heuristic_nodes.add(start);

		// Reserves memory for the current node and a spot for the newly created
		// nodes for the lineOfSight points
		Node current, temp;
		// Reserves memory for the current angle and sets it at the robot's
		// initial value
		double current_angle = initial_angle;
		// Reserves memory for change in angle, change in x, change in y, and
		// the distance between two points
		double d_angle = 0, dx = 0, dy = 0, distance = 0;

		// Continues to run until the polled node in the priority queue is the
		// end point
		while (true) {
			current = heuristic_nodes.poll(); // Pulls the node with the lowest
												// heuristic cost out

			// Stop the loop if the end point is pulled
			if (current == end) {
				current.parent_distance = Math
						.sqrt(Math.pow(current.x - current.parent.x, 2) + Math.pow(current.y - current.parent.y, 2));

				dy = current.y - current.parent.y;
				dx = current.x - current.parent.x;

				if (dx == 0) {
					current.parent_angle_change = Math.PI / 2;
					if (dy < 0)
						current.parent_angle_change *= -1;
				} else {
					current.parent_angle_change = Math.atan(dy / dx);

					if (dx < 0) {
						if (dy < 0)
							current.parent_angle_change -= Math.PI;
						else if (dy > 0)
							current.parent_angle_change += Math.PI;
					}
					current.parent_angle_change *= -1;
				}

				break;
			}

			// Removes the polled node from the open nodes array, populates the
			// paths array with nodes that the current node has a direct path to
			open_nodes.remove(current);
			// current.paths = lineOfSight(current, open_nodes, obstacles);

			current_angle += current.parent_angle_change;
			
			ArrayList<Node> los = lineOfSight(current, open_nodes, obstacles);
			open_nodes.removeAll(los);
			// Runs through all of the nodes the current node has a direct path
			// to
			for (Node p : los) {
				// p = new Node(p.x, p.y);

				// heuristic is the distance it is from the end_point divided by
				// the speed of the robot
				p.heuristic = Math.sqrt(Math.pow(end_point.x - p.x, 2) + Math.pow(end_point.y - p.y, 2))
						/ AStarPolygonal.lateral_speed;
				p.heuristic += current.cost;

				// Determine the change in angle
				dy = p.y - current.y;
				dx = p.x - current.x;
				distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

				// if (!(distance == 0)) {
				// d_angle = Math.acos(dx / distance);
				// d_angle *= dy < 0 ? -1 : 1; // if y is negative, make
				// // angle negative.
				// } else
				// d_angle = 0;

				// if the angle change is straight up or down
				if (dx == 0) {
					d_angle = Math.PI / 2;
					if (dy < 0)
						d_angle *= -1;
				} else {
					d_angle = Math.atan(dy / dx);

					if (dx < 0) {
						if (dy < 0)
							d_angle -= Math.PI;
						else if (dy > 0)
							d_angle += Math.PI;
					}

					d_angle *= -1;
				}

				// Total cost is the heuristic plus the cost to travel to the
				// node
				p.cost = p.heuristic + (distance / AStarPolygonal.lateral_speed)
						+ Math.abs(d_angle / AStarPolygonal.turn_speed);

				// This is for tracing back the path later
				p.parent = current;
				p.parent_angle_change = d_angle;
				p.parent_distance = distance;

				// current_angle += dy < 0 ? -1 * d_angle : d_angle;
				// current_angle += d_angle;

				// Adds the node to the heuristic nodes array
				heuristic_nodes.add(p);
			}
		}

		ArrayList<Node> travel_path = new ArrayList<Node>();
		travel_path.add(current);
		while (current.parent != null) {
			travel_path.add(0, current.parent);
			current = current.parent;
		}

		return travel_path;
	}

	/**
	 * Checks if the given points are valid.
	 * 
	 * @param p
	 * @param obstacles
	 * @return Returns true if they are not in an obstacle, false if they are.
	 */
	private static boolean checkValid(Point p, ArrayList<Polygon> obstacles) {
		// Preliminary scan of the obstacle's domain and range
		for (Polygon a : obstacles) {
			// The reason for doing this is so that it doesn't have to do the
			// whole system of equalities thing every time

			// If it is inside the obstacle's "box"
			if (a.checkInsideEst(p)) {

				// if it is in the "box", check the actual edges of the obstacle
				if (a.checkInsidePrecise(p))
					return false;
			}
		}

		return true;
	}

	/**
	 * -------------Not finished------------------ Takes in a point and returns
	 * all of the points it can travel to directly
	 * 
	 * @param p
	 * @param nodes
	 * @return
	 */
	private static ArrayList<Node> lineOfSight(Node p, ArrayList<Node> open_nodes, ArrayList<Polygon> obstacles) {
		ArrayList<Node> visible = new ArrayList<Node>();

		if (obstacles.isEmpty())
			return open_nodes;

		Polygon parent_polygon = null;
		int skip1 = -1, skip2 = -1;
		for (Polygon b : obstacles) {
			for (int j = 0; j < b.vertices.length; j++) {
				if (p.checkLoc(b.vertices[j])) {
					parent_polygon = b;

					if (j == 0)
						skip1 = b.vertices.length - 1;
					else
						skip1 = j - 1;

					if (j == b.vertices.length - 1)
						skip2 = 0;
					else
						skip2 = j + 1;
				}
			}
		}

		// I've got a feeling this is incredibly inefficient
		// Runs through all of the open nodes
		boolean blocked = false;
		// ArrayList<Node> share_obstacle = new ArrayList<Node>();
		for (Node a : open_nodes) {

			// So that we don't get paths inside of the obstacle
			if (parent_polygon != null) {
				for (int k = 0; k < parent_polygon.vertices.length; k++) {
					if (a.checkLoc(parent_polygon.vertices[k]) && k != skip1 && k != skip2) {
						blocked = true;
						break;
					}
				}
			}

			if (!blocked) {
				// Runs through all of the obstacles
				for (Polygon obs : obstacles) {

					// Runs through all of the vertices of the obstacles
					for (int i = 0; i < obs.vertices.length - 1; i++) {

						// Makes sure the nodes being checked aren't part of
						// this
						// edge.
						if (!(p.checkLoc(obs.vertices[i]) || p.checkLoc(obs.vertices[i + 1])
								|| a.checkLoc(obs.vertices[i]) || a.checkLoc(obs.vertices[i + 1]))) {
							// If the path from the given point intersects an
							// obstacle's edge, set blocked = true;
							if (lineIntersect(p, a, obs.vertices[i], obs.vertices[i + 1]))
								blocked = true;
						}
					}

					// Makes sure the nodes being checked aren't part of this
					// edge
					if (!(p.checkLoc(obs.vertices[0]) || p.checkLoc(obs.vertices[obs.vertices.length - 1])
							|| a.checkLoc(obs.vertices[0]) || a.checkLoc(obs.vertices[obs.vertices.length - 1]))) {
						// checks if path from given point intersects the edge
						// created by the first and last points in the vertices
						// array.
						if (lineIntersect(p, a, obs.vertices[obs.vertices.length - 1], obs.vertices[0]))
							blocked = true;
					}

				}
			}

			if (!blocked)
				visible.add(a);

			blocked = false;
		}

		return visible;
	}

	/**
	 * Checks if two line segments P (determined by points p1 and p2) and Q (q1,
	 * q2) intersect.
	 * 
	 * @param p1
	 * @param p2
	 * @param q1
	 * @param q2
	 * @return True if intersection, false if not.
	 */
	public static boolean lineIntersect(Point p1, Point p2, Point q1, Point q2) {

		float denominator = ((p2.x - p1.x) * (q2.y - q1.y)) - ((p2.y - p1.y) * (q2.x - q1.x));

		if (denominator == 0) {
			return false;
		}

		float numerator1 = ((p1.y - q1.y) * (q2.x - q1.x)) - ((p1.x - q1.x) * (q2.y - q1.y));

		float numerator2 = ((p1.y - q1.y) * (p2.x - p1.x)) - ((p1.x - q1.x) * (p2.y - p1.y));

		if (numerator1 == 0 || numerator2 == 0) {
			return false;
		}

		float r = numerator1 / denominator;
		float s = numerator2 / denominator;

		return (r > 0 && r < 1) && (s > 0 && s < 1);

	}

	private static ArrayList<Polygon> createObstacles(ArrayList<ArrayList<int[]>> points) {
		Point[] temp;
		ArrayList<Polygon> obstacles = new ArrayList<>();

		for (List<int[]> obs : points) {
			temp = new Point[obs.size()];
			for (int i = 0; i < obs.size(); i++) {
				temp[i] = new Point(obs.get(i)[0], obs.get(i)[1]);
			}
			obstacles.add(new Polygon(temp));
		}

		return obstacles;
	}
}