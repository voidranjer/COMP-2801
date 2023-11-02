import java.util.ArrayList;
import java.awt.Point;

public class Obstacle {
  private ArrayList<Point>   vertices;
  
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
}