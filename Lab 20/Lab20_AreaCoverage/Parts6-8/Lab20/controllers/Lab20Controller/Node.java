import java.io.*;
import java.util.ArrayList;
import java.awt.Graphics;
import java.awt.Color;
import java.awt.Point;

public class Node implements Comparable<Node> {
  public static int         RADIUS = 3;

  // These are the instance variables
  private String            label;           // text for the node (unused)
  private Point     	      location;        // location in the world
  private ArrayList<Edge>   incidentEdges;   // list of all edges that connect to this Node
  private boolean   	      visited;         // true if the node has been visited
  private boolean   	      selected;        // true if the node is selected (i.e., highlighted)
  
  // This keeps track of the obstacle that it came from ... needed for visibility graph
  private Obstacle          obstacle;
  
  // These are used for shortest path calculations
  private float             distance;        // Distance (in the grapg) from the source location
  private Node              previous;        // Previous node along the shortest path

  // Some constructors
  public Node() { this("", new Point(0,0)); }
  public Node(String aLabel) { this(aLabel, new Point(0,0)); }
  public Node(Point aPoint) { this("", aPoint); }
  public Node(Point aPoint, Obstacle ob) { this("", aPoint); obstacle = ob; }
  public Node(String aLabel, Point aPoint) {
    label = aLabel;
    location = aPoint;
    incidentEdges = new ArrayList<Edge>();
    previous = null;
    distance = -1;    // not calculated yet
    selected = false;
    obstacle = null;
  }

  // The get & set methods
  public String getLabel() { return label; }
  public Point getLocation() { return location; }
  public float getDistance() { return distance; }
  public Node getPrevious() { return previous; }
  public Obstacle getObstacle() { return obstacle; }
  public boolean hasBeenVisited() { return visited; }

  public void setLabel(String newLabel) { label = newLabel; }
  public void setLocation(Point aPoint) { location = aPoint; }
  public void setLocation(int x, int y) { location = new Point(x, y); }
  public void setDistance(float d) { distance = d; }
  public void setPrevious(Node p) { previous = p; }
  public void setVisited(boolean b) { visited = b; }

  // Returns all graph Edges connected to this Node
  public ArrayList<Edge> incidentEdges() { return incidentEdges; }
  
  // Add the given graph Edge to the list of incident edges for this Node
  public void addIncidentEdge(Edge e) { incidentEdges.add(e); }

  public boolean isSelected() { return selected; }
  public void setSelected(boolean state) { selected = state; }
  public void toggleSelected() { selected = !selected; }

  // Returns a list of all Nodes that are connected to this one via an edge
  public ArrayList<Node> neighbours() {
    ArrayList<Node> result = new ArrayList<Node>();
    for (Edge e: incidentEdges)
      result.add(e.otherEndFrom(this));
    return result;
  }

  // Get te edge that connects to this neighbour
  public Edge getNeighbouringEdgeTo(Node n) {
   for (Edge e: incidentEdges)
     if (e.otherEndFrom(this) == n)
       return e;
   return null;
  }
   
  // Nodes look like this:  label(12,43)
  public String toString() {
    return(label + "(" + location.x + "," + location.y + ")");
  }

  // Draw the Node at the specified magnification factor with the given Pen on a panel with the given height
  public void draw(Graphics aPen, int height, int magnification) {
    int radius = RADIUS;
    
    if (selected) {
      aPen.setColor(Color.red);
      radius = RADIUS+2;
    }
    else
      aPen.setColor(Color.yellow);

    // Draw a blue-filled circle around the center of the node
    aPen.fillOval(location.x*magnification - radius, height - location.y*magnification - radius, radius * 2, radius * 2);

    // Draw a black border around the circle
    aPen.setColor(Color.black);
    aPen.drawOval(location.x*magnification - radius, height - location.y*magnification - radius, radius * 2, radius * 2);

    // Draw a label at the top right corner of the node
    if (label != "")
      aPen.drawString(label, location.x*magnification + radius, height - location.y*magnification - radius);
  }

  // Return true if the Node is equal another Node passed in
  public boolean equals(Object obj) {
    if (obj == null) return false;
    if (!(obj instanceof Node)) return false;

    Node n = (Node)obj;
    if (location == null) return false;
    if (n.getLocation() == null) return false;
    return (location.x == n.getLocation().x) && (location.y == n.getLocation().y);
  }

  // Compare two Nodes assuming one node comes before another if its distance attribute is smaller
  public int compareTo(Node n) {
    if (!(n instanceof Node)) return 0;

    if (distance < n.distance) return -1;
    if (distance > n.distance) return 1;
    return 0;
  }
  
  // Required for Comparable interface
  public int hashCode() { return location.hashCode(); }
}