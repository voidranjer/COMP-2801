import java.util.ArrayList;
import java.awt.Point;
import java.awt.Polygon;

public class Obstacle {
  private ArrayList<Point>   vertices;   // An obstacle is just its vertices as points stored in CCW order
  private Polygon            asPolygon;  // A polygonal representation of the obstacle for display.

  
  public Obstacle() {
    vertices = new ArrayList<Point>();
  }
  
  public ArrayList<Point> getVertices() { return vertices; }
  

  public void addVertex(int x, int y) {
    vertices.add(new Point(x,y));
  }
  
  public Point getVertex(int i) {
    return vertices.get(i);
  }
  
  public int numVertices() { return vertices.size(); }
  
  // Compute and store a polygon version of this obstacle, which makes things easier 
  // for displaying and for checking point inclusion
  public void computePolygon() {
    asPolygon = new Polygon();
    for (Point v: vertices)
      asPolygon.addPoint(v.x, v.y);
  }
  
  // Return true if the given point lies on or within the obstacle boundary.
  
  public boolean contains(Point p) {
    if (asPolygon == null)  
      return false;
    return asPolygon.contains(p);
  }
}