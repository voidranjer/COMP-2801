import java.util.ArrayList;
import java.awt.*;

public class PathPlanner {
  private static final Color OBSTACLE_COLOR = new Color(150, 200, 255); // light pale blue
  private static final Color OBSTACLE_ID_COLOR = new Color(150, 0, 0); // Dark red

  private Point start; // Start point of robot
  private Point end; // End point of robot
  private VectorMap vmap; // Needs to be loaded
  private ArrayList<Point> supportLines; // Needs to be computed. Support lines from start location only
  private Graph visibilityGraph; // Needs to be computed

  public static boolean ShowObstacles = true;
  public static boolean ShowStartDest = true;
  public static boolean ShowSupportLines = false;
  public static boolean ShowGraph = false;
  public static boolean ShowObstacleLabels = false;

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
    end = new Point(x, y);
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
  public boolean supportLineIntersectsObstacle(int x, int y, int supportX, int supportY,
      ArrayList<Obstacle> allObstacles) {

    for (Obstacle ob : allObstacles) {
      for (int i = 0; i < ob.numVertices(); i++) {
        Point v = ob.getVertex(i);
        Point va = ob.getVertex((i + 1) % ob.numVertices()); // vertex of obstacle after v

        if (java.awt.geom.Line2D.Double.linesIntersect(x, y, supportX, supportY, v.x, v.y, va.x, va.y))
          return true;
      }
    }

    return false; // no intersection
  }

  private boolean isSupportVertex(int t1, int t2) {
    // From Slide 5

    // PL = pi
    if (t1 <= 0 && t2 <= 0)
      return true;

    // PR = pi
    if (t1 >= 0 && t2 >= 0)
      return true;

    return false;
  }

  public ArrayList<Point> computeSupportPointsFrom(ArrayList<Obstacle> allObstacles, int x, int y) {
    ArrayList<Point> supports = new ArrayList<Point>();

    for (Obstacle ob : allObstacles) {
      for (int i = 0; i < ob.numVertices(); i++) {
        Point v = ob.getVertex(i);

        int xs = x;
        int ys = y;
        int xi = v.x;
        int yi = v.y;
        int xip1 = ob.getVertex((i + 1) % ob.numVertices()).x;
        int yip1 = ob.getVertex((i + 1) % ob.numVertices()).y;
        int xim1 = ob.getVertex((i - 1 + ob.numVertices()) % ob.numVertices()).x;
        int yim1 = ob.getVertex((i - 1 + ob.numVertices()) % ob.numVertices()).y;

        int t1 = (xi - xs) * (yip1 - ys) - (yi - ys) * (xip1 - xs);
        int t2 = (xi - xs) * (yim1 - ys) - (yi - ys) * (xim1 - xs);

        // Special Case 1 - Slide 11
        if (xs == xi && ys == yi) {
          supports.add(v);
          continue;
        }

        if (isSupportVertex(t1, t2)) {
          if (!supportLineIntersectsObstacle(xs, ys, xi, yi, allObstacles))
            supports.add(v);
        }

      }
    }

    if (!supportLineIntersectsObstacle(start.x, start.y, end.x, end.y, allObstacles))
      supports.add(end);

    return supports;
  }

  // Create and store a new visibility graph for the given vector map
  public void computeVisibilityGraph() {
    ArrayList<Obstacle> allObstacles = vmap.getObstacles(); // all obstacles in environment

    // Create and store the graph for display access later
    visibilityGraph = new Graph();

    // WRITE YOUR CODE HERE

  }

  // This procedure displays the results of the PathPlanning algorithm which
  // includes, the obstacles, support lines and visibility graph
  public void displayResults(Graphics aPen, double resolution, Point mousePosition) {
    int magnification = (int) (1 / resolution);
    try {
      if (ShowObstacles) {
        // Convert obstacles to polygons, then display the polygons
        int count = 0;
        for (Obstacle ob : vmap.getObstacles()) {
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
        supportLines = computeSupportPointsFrom(vmap.getObstacles(), visPoint.x, visPoint.y);
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
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}