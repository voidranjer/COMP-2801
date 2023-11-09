import java.util.ArrayList;
import java.awt.Point;
import java.awt.Polygon;

public class Obstacle {
  private ArrayList<Point> vertices; // An obstacle is just its vertices as points stored in CCW order
  private Polygon asPolygon; // A polygonal representation of the obstacle for display.
  private boolean polygonWasComputed; // true if a polygonal version of this obstacle was computed

  public Obstacle() {
    vertices = new ArrayList<Point>();
    polygonWasComputed = false;
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
      return false;
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

  private boolean isLeftTurn(Point prev, Point curr, Point next) {
    return ((curr.x - prev.x) * (next.y - prev.y) - (curr.y - prev.y) * (next.x -
        prev.x)) > 0;
  }

  // Check if the obstacle is convex
  public boolean isConvex() {
    if (vertices.size() < 3)
      return false;

    for (int i = 0; i < vertices.size(); i++) {
      // System.out.println("---------" + vertices.size() + "----------");
      // System.out.println("i - 1: " + prevIndex);
      // System.out.println("i: " + i);
      // System.out.println("i + 1: " + (i + 1) % vertices.size());

      Point prev = vertices.get((i - 1 + vertices.size()) % vertices.size());
      Point curr = vertices.get(i);
      Point next = vertices.get((i + 1) % vertices.size());

      if (!isLeftTurn(prev, curr, next)) {
        return false;
      }
    }

    return true;
  }

  // Decompose an obstacle into triangles. Return an arraylist of Obstacles where
  // each obstacle is a triangle.
  public ArrayList<Obstacle> splitIntoTriangles() {
    Obstacle ear = null;
    ArrayList<Obstacle> triangles = new ArrayList<Obstacle>();

    if (vertices.size() < 3) {
      triangles.add(this);
      return triangles;
    }

    ArrayList<Point> q = new ArrayList<>();
    for (int i = 0; i < vertices.size(); i++) {
      q.add(new Point(vertices.get(i)));
    }

    while (q.size() > 3) {
      int earIndex = -1;

      for (int i = 0; i < q.size(); i++) {
        Point prev = q.get((i - 1 + q.size()) % q.size());
        Point curr = q.get(i);
        Point next = q.get((i + 1) % q.size());

        if (isLeftTurn(prev, curr, next)) {
          ear = new Obstacle();
          ear.addVertex(prev.x, prev.y);
          ear.addVertex(curr.x, curr.y);
          ear.addVertex(next.x, next.y);

          earIndex = i;

          for (int k = 0; k < q.size(); k++) {
            if (k == i - 1 || k == i || k == i + 1)
              continue;

            Point p = q.get(k);
            if (ear.contains(p) || ear.pointOnBoundary(p)) {
              earIndex = -1;
            }
          }
          if (earIndex != -1)
            break;
        }

      }

      if (earIndex == -1)
        return triangles;

      q.remove(earIndex);
      triangles.add(ear);

    }

    Obstacle lastObstacle = new Obstacle();
    lastObstacle.addVertex(q.get(0).x, q.get(0).y);
    lastObstacle.addVertex(q.get(1).x, q.get(1).y);
    lastObstacle.addVertex(q.get(2).x, q.get(2).y);
    triangles.add(lastObstacle);

    return triangles;
  }
}