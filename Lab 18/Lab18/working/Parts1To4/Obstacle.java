// James Yap 101276054
// Jason Wu 101233618

import java.util.ArrayList;
import java.awt.Point;
import java.awt.Polygon;

public class Obstacle {
  private ArrayList<Point> vertices; // An obstacle is just its vertices as points stored in CCW order
  private Polygon asPolygon; // A polygonal representation of the obstacle for display.
  private boolean polygonWasComputed; // true if a polygonal version of this obstacle was computed
  private Obstacle original; // NULL if this is an original obstacle and the original obstacle that it was
                             // part of if this was not

  public Obstacle() {
    vertices = new ArrayList<Point>();
    polygonWasComputed = false;
    original = this;
  }

  public ArrayList<Point> getVertices() {
    return vertices;
  }

  public void addVertex(int x, int y) {
    vertices.add(new Point(x, y));
    polygonWasComputed = false;
    computePolygon();
  }

  public Point getVertex(int i) {
    return vertices.get(i);
  }

  public int numVertices() {
    return vertices.size();
  }

  public void setOriginalObstacle(Obstacle ob) {
    original = ob;
  }

  public Obstacle getOriginalObstacle() {
    return original;
  }

  // Return a polygonal version of the obstacle
  public Polygon asPolygon() {
    if (polygonWasComputed)
      return asPolygon;
    computePolygon();
    return asPolygon;
  }

  // Compute and store a polygon version of this obstacle, which makes things
  // easier
  // for displaying and for checking point inclusion
  public void computePolygon() {
    if (polygonWasComputed)
      return;
    asPolygon = new Polygon();
    for (Point v : vertices)
      asPolygon.addPoint(v.x, v.y);
    polygonWasComputed = true;
  }

  // Return true if the given point lies on or within the obstacle boundary.
  public boolean contains(Point p) {
    if (asPolygon == null)
      computePolygon();
    return asPolygon.contains(p);
  }

  // Determines whether or not the given point is on the boundary of the obstacle
  public boolean pointOnBoundary(Point p) {
    boolean onBoundary = false;
    for (int i = 0; i < vertices.size(); i++) {
      Point v1 = vertices.get(i);
      Point v2 = vertices.get((i + 1) % vertices.size());
      if (java.awt.geom.Line2D.Double.linesIntersect(p.x, p.y, p.x, p.y, v1.x, v1.y, v2.x, v2.y))
        onBoundary = true;
    }
    return onBoundary;
  }

  // Check if the obstacle is convex
  public boolean isConvex() {
    for (int i = 0; i < vertices.size(); i++) {
      int im1 = (i - 1 + vertices.size()) % vertices.size();
      int ip1 = (i + 1) % vertices.size();
      Point pim1 = vertices.get(im1);
      Point pi = vertices.get(i);
      Point pip1 = vertices.get(ip1);

      if (((pi.x - pim1.x) * (pip1.y - pim1.y) - (pi.y - pim1.y) * (pip1.x - pim1.x)) <= 0)
        return false;
    }
    return true;
  }

  // Decompose an obstacle into triangles. Return an arraylist of Obstacles where
  // each obstacle is a triangle.
  public ArrayList<Obstacle> splitIntoTriangles() {
    ArrayList<Obstacle> triangles = new ArrayList<Obstacle>();

    if (vertices.size() <= 3) {
      triangles.add(this);
      return triangles;
    }

    // Copy the points of the obstacle
    ArrayList<Point> points = new ArrayList<Point>();
    int count = 0;
    for (Point p : vertices)
      points.add(new Point(p.x, p.y));

    // Keep cutting off ears
    Obstacle ear = null;
    while (points.size() > 3) {
      int earIndex = -1;
      // First find an ear (assumes CCW vertex ordering)
      for (int i = 0; i < points.size(); i++) {
        int im1 = (i - 1 + points.size()) % points.size();
        int ip1 = (i + 1) % points.size();
        Point pim1 = points.get(im1);
        Point pi = points.get(i);
        Point pip1 = points.get(ip1);
        if (((pi.x - pim1.x) * (pip1.y - pim1.y) - (pi.y - pim1.y) * (pip1.x - pim1.x)) > 0) {
          ear = new Obstacle();
          ear.setOriginalObstacle(this);
          ear.addVertex(pim1.x, pim1.y);
          ear.addVertex(pi.x, pi.y);
          ear.addVertex(pip1.x, pip1.y);
          // Make sure no other points are inside of it
          earIndex = (i) % points.size();
          for (int j = 0; j < points.size(); j++) {
            if ((j != i) && (j != im1) && (j != ip1)) {
              if (ear.contains(points.get(j)) || ear.pointOnBoundary(points.get(j))) {
                earIndex = -1;
                break;
              }
            }
          }
          if (earIndex != -1)
            break;
        }
      }
      // Now remove the vertex at the ear index and add the ear
      if (earIndex != -1) {
        points.remove(earIndex);
        triangles.add(ear);
      } else // If there were no ears remaining, quit with whatever triangles have been found
        return triangles;
    }
    // The last one
    ear = new Obstacle();
    ear.setOriginalObstacle(this);
    ear.addVertex(points.get(0).x, points.get(0).y);
    ear.addVertex(points.get(1).x, points.get(1).y);
    ear.addVertex(points.get(2).x, points.get(2).y);
    triangles.add(ear);

    return triangles;
  }

  // List the vertices of the Obstacle
  public void listVertices() {
    for (Point p : vertices)
      System.out.println("(" + p.x + "," + p.y + ")");
  }

  // Determine the convex hull of a set of points
  private Obstacle convexHull(ArrayList<Point> points) {
    Obstacle hull = new Obstacle(); // Do NOT remove this
    hull.setOriginalObstacle(this.getOriginalObstacle()); // Do NOT remove this

    if (points.isEmpty()) // Do NOT remove this
      return hull; // Do NOT remove this

    // ADD YOUR CODE HERE

    // get rightmost point
    Point pHull = points.get(0);
    for (Point p : points) {
      if (p.x > pHull.x)
        pHull = p;
    }

    Point pStart = pHull;
    Point pEnd = null;

    while (!pStart.equals(pEnd)) {
      hull.addVertex(pHull.x, pHull.y);
      pEnd = points.get(0);
      for (Point pi : points) {
        if (pHull.equals(pEnd)) {
          pEnd = pi;
        }

        boolean isRightTurn = ((pEnd.x - pHull.x) * (pi.y - pHull.y) - (pEnd.y - pHull.y) * (pi.x - pHull.x) < 0);
        if (isRightTurn) {
          pEnd = pi;
        }
      }
      pHull = pEnd;
    }

    return hull; // Do NOT remove this
  }

  // Grow the obstacle by the amount specified by the given radius, using the
  // degree units
  // specified for each corner. Return the grown obstacle.
  public Obstacle grow(float radius, int degreeRoundnessAngle) {
    ArrayList<Point> points = new ArrayList<Point>(); // Do NOT remove this

    // ADD YOUR CODE HERE
    final double DEGREE_UNIT = degreeRoundnessAngle;
    final double d = radius / cos(DEGREE_UNIT / 2.0);

    double xoff = -1;
    double yoff = -1;
    double pX = -1;
    double pY = -1;
    double x = -1;
    double y = -1;

    for (Point vertex : vertices) {
      x = vertex.getX();
      y = vertex.getY();
      for (double a = 0; a < 360; a += DEGREE_UNIT) {
        xoff = d * cos(a);
        yoff = d * sin(a);

        if (xoff > 0)
          pX = x + Math.ceil(xoff);
        else
          pX = x + Math.floor(xoff);

        if (yoff > 0)
          pY = y + Math.ceil(yoff);
        else
          pY = y + Math.floor(yoff);

        points.add(new Point((int) pX, (int) pY));
      }
    }

    // Obstacle hull = convexHull(points);
    // for (Point vertex : points) {
    // hull.addVertex(vertex.x, vertex.y);
    // }
    // return hull;
    return convexHull(points); // Do NOT remove this
  }

  private double cos(double angle) {
    return Math.cos(Math.toRadians(angle));
  }

  private double sin(double angle) {
    return Math.sin(Math.toRadians(angle));
  }
}