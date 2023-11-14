import java.util.ArrayList;
import java.util.PriorityQueue;
import java.awt.*;

public class PathPlanner {
  private static final Color OBSTACLE_COLOR = new Color(150, 200, 255); // light pale blue
  private static final Color OBSTACLE_ID_COLOR = new Color(150, 0, 0); // Dark red

  private Point start; // Start point of robot
  private Point end; // End point of robot
  private Point originalEnd; // Copy of the original end point (since mouse position may update it)
  private VectorMap vmap; // Needs to be loaded
  private ArrayList<Point> supportLines; // Needs to be computed. Support lines from start location only
  private Graph visibilityGraph; // Needs to be computed

  public static boolean ShowObstacles = true;
  public static boolean ShowConvexObstacles = false;
  public static boolean ShowStartDest = true;
  public static boolean ShowSupportLines = false;
  public static boolean ShowGraph = false;
  public static boolean ShowObstacleLabels = false;
  public static boolean ShowPath = false;
  public static boolean ShowDynamicPath = false;
  public static boolean ShowTree = false;
  public static boolean MinimizeTurns = false;

  // A Path Planner also requires a starting and end location, but these will be
  // set later
  public PathPlanner(VectorMap vm) {
    start = null; // Needs to be provided later
    end = null; // Needs to be provided later
    vmap = vm;
    supportLines = null; // Needs to be computed
    visibilityGraph = null; // Needs to be computed
  }

  // These are used to refresh the start or end points or the map
  public void setStart(int x, int y) {
    start = new Point(x, y);
  }

  public void setEnd(int x, int y) {
    if (end == null)
      originalEnd = new Point(x, y);
    end = new Point(x, y);
  }

  public void resetEndpoint() {
    end = originalEnd;
  }

  public void setVectorMap(VectorMap m) {
    vmap = m;
  }

  public Graph getVisibilityGraph() {
    return visibilityGraph;
  }

  // Returns true if a support line from (x,y) to (supportX, supportY) intersects
  // an obstacle
  // at a point other than at a vertex.
  public boolean supportLineIntersectsObstacle(int x, int y, int supportX, int supportY, Obstacle fromObst,
      ArrayList<Obstacle> allObstacles) {
    for (Obstacle ob : allObstacles) {
      for (int i = 0; i < ob.numVertices(); i++) {
        Point v1 = ob.getVertex(i);
        Point v2 = ob.getVertex((i + 1) % ob.numVertices());

        if (fromObst != ob) {
          // Make sure that the (x,y) and (supportX, supportY) are not vertices of the
          // edge being checked
          if ((x == v1.x) && (y == v1.y) && (supportX == v2.x) && (supportY == v2.y))
            return true;
          if ((x == v2.x) && (y == v2.y) && (supportX == v1.x) && (supportY == v1.y))
            return true;

          // Since it is not attached to this edge being checked, make sure there is no
          // intersection
          if (!((x == v1.x) && (y == v1.y)) && !((x == v2.x) && (y == v2.y)) &&
              !((supportX == v1.x) && (supportY == v1.y)) && !((supportX == v2.x) && (supportY == v2.y)))
            if (java.awt.geom.Line2D.Double.linesIntersect(x, y, supportX, supportY, v1.x, v1.y, v2.x, v2.y))
              return true;
        }
      }
    }
    return false; // Since there was no intersection
  }

  // Compute and return a list of the Visible Support Points with respect to the
  // given point
  public ArrayList<Point> computeSupportPointsFrom(ArrayList<Obstacle> allObstacles, int x, int y) {
    ArrayList<Point> supports = new ArrayList<Point>();
    Point intersection = null;

    // Go through the polygons and find the support points

    for (Obstacle ob : allObstacles) {
      Point PL = null, PR = null; // Assume no supports to start

      for (int i = 0; i < ob.numVertices(); i++) {
        Point vi = ob.getVertex(i);
        Point bvi = ob.getVertex((i - 1 + ob.numVertices()) % ob.numVertices());
        Point avi = ob.getVertex((i + 1) % ob.numVertices());

        if ((x == vi.x) && (y == vi.y)) {
          PL = new Point(bvi.x, bvi.y);
          if (!supportLineIntersectsObstacle(x, y, PL.x, PL.y, ob, allObstacles))
            supports.add(PL);
          PR = new Point(avi.x, avi.y);
          if (!supportLineIntersectsObstacle(x, y, PR.x, PR.y, ob, allObstacles))
            supports.add(PR);
          continue;
        }
        // Calculate the turn types
        double t1 = (vi.x - x) * (avi.y - y) - (vi.y - y) * (avi.x - x);
        double t2 = (vi.x - x) * (bvi.y - y) - (vi.y - y) * (bvi.x - x);

        if ((t1 <= 0) && (t2 <= 0)) {
          PL = new Point(vi.x, vi.y);
          if (!supportLineIntersectsObstacle(x, y, PL.x, PL.y, ob, allObstacles))
            supports.add(PL);
        }
        if ((t1 >= 0) && (t2 >= 0)) {
          PR = new Point(vi.x, vi.y);
          if (!supportLineIntersectsObstacle(x, y, PR.x, PR.y, ob, allObstacles))
            supports.add(PR);
        }
      }
    }

    // Now add a line from the start to the destination if it does not intersect any
    // polygons
    Point END = new Point(end.x, end.y);
    if (!supportLineIntersectsObstacle(x, y, END.x, END.y, null, allObstacles))
      supports.add(END);

    return supports;
  }

  // Create and store a new visibility graph for the set of convex obstacles
  public void computeVisibilityGraph() {
    ArrayList<Obstacle> allConvexObstacles = vmap.getConvexObstacles();

    ArrayList<Point> supports;

    // Create and store the graph for display access later
    visibilityGraph = new Graph();

    // Add the start & end points as graph nodes
    Node s = new Node("", start);
    visibilityGraph.addNode(s);
    Node e = new Node("", end);
    visibilityGraph.addNode(e);

    // Add all the obstacle vertices as graph nodes
    for (Obstacle ob : allConvexObstacles) {
      for (int i = 0; i < ob.numVertices(); i++) {
        Point p = new Point(ob.getVertex(i).x, ob.getVertex(i).y);
        // Check to make sure that the vertex is not a duplicate
        if (visibilityGraph.node(p.x, p.y) == null)
          visibilityGraph.addNode(new Node("", p));
      }
    }

    // Compute the support lines from the start location
    supports = computeSupportPointsFrom(allConvexObstacles, start.x, start.y);
    supportLines = supports;
    for (Point p : supports) {
      Node m = visibilityGraph.node(p.x, p.y);
      if (m != null)
        visibilityGraph.addEdge(s, m);
    }

    // Compute the support lines from all graph nodes corresponding to obstacle
    // vertices
    for (Node n : visibilityGraph.getNodes()) {
      if ((n == s) || (n == e))
        continue;
      supports = computeSupportPointsFrom(allConvexObstacles, n.getLocation().x, n.getLocation().y);
      for (Point p : supports) {
        Node m = visibilityGraph.node(p.x, p.y);
        if (m != null)
          visibilityGraph.addEdge(n, m);
      }
    }
  }

  // This code runs Dijkstra's shortest path algorithm in the visibilty graph
  private void runDijkstrasAlgorithmFrom(Node s) {

    // WRITE YOUR CODE HERE FOR PART 2

    PriorityQueue<Node> q = new PriorityQueue<Node>();

    s.setDistance(0);

    for (Node n : visibilityGraph.getNodes()) {
      if (!n.equals(s))
        n.setDistance(Float.POSITIVE_INFINITY);
      n.setPrevious(null);
      q.add(n);
    }

    while (!q.isEmpty()) {
      Node v = q.remove();
      // if (v.equals(end))
      // return;

      for (Node u : v.neighbours()) {
        Edge vu = visibilityGraph.edge(v, u);
        float newDistance = v.getDistance() + vu.length();
        if (u.getDistance() > newDistance) {
          u.setPrevious(v);
          u.setDistance(newDistance);

          // Re-sort node u in Q (because a weight has changed now)
          q.remove(u);
          q.add(u);
        }
      }
    }

  }

  // This code runs Dijkstra's shortest path algorithm in the visibilty graph with
  // a high cost on turns
  private void runShortestTurnPathFrom(Node s) {

    // WRITE YOUR CODE HERE FOR PART 4

    PriorityQueue<Node> q = new PriorityQueue<Node>();

    s.setDistance(0);

    for (Node n : visibilityGraph.getNodes()) {
      if (!n.equals(s))
        n.setDistance(Float.POSITIVE_INFINITY);
      n.setPrevious(null);
      q.add(n);
    }

    while (!q.isEmpty()) {
      Node v = q.remove();

      for (Node u : v.neighbours()) {
        float newDistance = v.getDistance() + 1;
        if (u.getDistance() > newDistance) {
          u.setPrevious(v);
          u.setDistance(newDistance);

          // Re-sort node u in Q (because a weight has changed now)
          q.remove(u);
          q.add(u);
        }
      }
    }

  }

  // Compute the shortest path tree in the visibility graph (assumed computed).
  // Highlight the
  // shortest path edges from the start to the end.
  public void computeShortestPath() {
    // First make sure that the visibility graph has been computed
    if (visibilityGraph == null)
      computeVisibilityGraph();

    // Make sure that there are start and end nodes in the graph before continuing.
    Node s = visibilityGraph.node(start.x, start.y);
    Node e = visibilityGraph.node(end.x, end.y);
    if ((s == null) || (e == null))
      return;
    e.setLabel("(" + end.x + "," + end.y + ")");
    s.setLabel("(" + start.x + "," + start.y + ")");

    // Compute the shortest path tree
    if (MinimizeTurns)
      runShortestTurnPathFrom(s);
    else
      runDijkstrasAlgorithmFrom(s);

    // Unselect all the nodes and edges
    for (Node n : visibilityGraph.getNodes())
      n.setSelected(false);
    for (Edge ed : visibilityGraph.getEdges())
      ed.setSelected(false);

    // Now trace the path back from the end node and select all edges and Nodes
    // along the path
    if (ShowPath || ShowDynamicPath) {
      Node n = e;
      while ((n != s) && (n != null)) {
        n.setSelected(true);
        // n.setLabel(""+n.getDistance());
        Node next = n.getPrevious();
        Edge ed = visibilityGraph.edge(n, next);
        if (ed != null)
          ed.setSelected(true);
        n = next;
      }
      s.setSelected(true);
    } else if (ShowTree) {
      for (Node n : visibilityGraph.getNodes()) {
        Edge ed = visibilityGraph.edge(n, n.getPrevious());
        if (ed != null)
          ed.setSelected(true);
      }
      s.setSelected(true);
    }
  }

  // This procedure displays the results of the PathPlanning algorithm which
  // includes, the obstacles, support lines and visibility graph
  public void displayResults(Graphics aPen, double resolution, Point mousePosition) {
    int magnification = (int) (1 / resolution);
    try {
      // Use the convex obstacles if that is what is wanted, otherwise use the
      // original obstacles
      ArrayList<Obstacle> obstaclesToDisplay = vmap.getObstacles();
      if (ShowConvexObstacles)
        obstaclesToDisplay = vmap.getConvexObstacles();

      if (ShowObstacles || ShowConvexObstacles) {
        // Convert obstacles to polygons, then display the polygons
        int count = 0;
        for (Obstacle ob : obstaclesToDisplay) {
          Polygon p = new Polygon();
          Point avg = new Point();
          for (int i = 0; i < ob.numVertices(); i++) {
            int x = MapperApp.MARGIN_X / 2 + ob.getVertex(i).x * magnification;
            int y = MapperApp.MARGIN_Y / 2 + vmap.getHeight() - ob.getVertex(i).y * magnification;
            p.addPoint(x, y);
            avg.x += x;
            avg.y += y;
          }
          aPen.setColor(OBSTACLE_COLOR);
          aPen.fillPolygon(p);
          aPen.setColor(Color.black);
          aPen.drawPolygon(p);

          // Show the obstacle number at its center
          if (ShowObstacleLabels) {
            aPen.setColor(OBSTACLE_ID_COLOR);
            aPen.drawString("" + count++, (avg.x / p.npoints) - 5, (avg.y / p.npoints) + 5);
          }
        }
      }
      if (ShowStartDest) {
        // Now draw the start/end points
        Node s = new Node(new Point(start.x, start.y));
        Node e = new Node(new Point(end.x, end.y));
        s.draw(aPen, vmap.getHeight(), magnification);
        e.draw(aPen, vmap.getHeight(), magnification);
      }
      if (ShowSupportLines) {
        aPen.setColor(Color.red);
        Point visPoint = new Point((int) ((mousePosition.x - MapperApp.MARGIN_X / 2) * resolution),
            (int) (vmap.getHeight() * resolution - (mousePosition.y - MapperApp.MARGIN_Y / 2) * resolution));
        supportLines = computeSupportPointsFrom(obstaclesToDisplay, visPoint.x, visPoint.y);
        for (Point p : supportLines) {
          if (p != null)
            aPen.drawLine(MapperApp.MARGIN_X / 2 + visPoint.x * magnification,
                MapperApp.MARGIN_Y / 2 + vmap.getHeight() - visPoint.y * magnification,
                MapperApp.MARGIN_X / 2 + p.x * magnification,
                MapperApp.MARGIN_Y / 2 + vmap.getHeight() - p.y * magnification);
        }
      }
      if (ShowGraph && (visibilityGraph != null)) {
        visibilityGraph.draw(aPen, vmap.getHeight(), magnification);
      }
      if (ShowDynamicPath) { // Recompute the graph and path in case the mouse location changed.
        // Make sure that endpoint does not lie within an obstacle before displaying
        boolean liesOutside = true;
        for (Obstacle ob : vmap.getObstacles())
          if (ob.pointOnBoundary(end) || ob.contains(end))
            liesOutside = false;
        if (liesOutside)
          visibilityGraph.drawSelectedOnly(aPen, vmap.getHeight(), magnification, 5); // 5 = Thicker lines
        Node e = new Node(new Point(end.x, end.y));
        e.setLabel("(" + end.x + "," + end.y + ")");
        e.draw(aPen, vmap.getHeight(), magnification);
        Node s = new Node(new Point(start.x, start.y));
        s.setLabel("(" + start.x + "," + start.y + ")");
        s.draw(aPen, vmap.getHeight(), magnification);
      }
      // This shows the shortest path
      else if (ShowPath && (visibilityGraph != null)) {
        visibilityGraph.drawSelectedOnly(aPen, vmap.getHeight(), magnification, 5); // 5 = Thicker lines
      } else if (ShowTree && (visibilityGraph != null)) {
        visibilityGraph.drawSelectedOnly(aPen, vmap.getHeight(), magnification, 2); // 2 = Thinner lines
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}