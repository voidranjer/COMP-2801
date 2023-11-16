import java.io.*;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.BasicStroke;
import java.awt.Color;

public class Edge {
  public static final int	WIDTH = 7;  // the thickness (in pixels) of selected edges

  // These are the instance variables
  private String   label;               // text for the node (unused)
  private Node     startNode, endNode;  // the start and end nodes that this graph edge connects
  private boolean  selected;            // true if the edge is selected (i.e., highlighted)
 
  // Some constructors.  Note that the default is not acceptable since we WANT nodes
  public Edge(Node start, Node end) {
    label = "";
    startNode = start;
    endNode = end;
  }

  public Edge(String aLabel, Node start, Node end) {
    label = aLabel;
    startNode = start;
    endNode = end;
  }

  // The get & set methods
  public String getLabel() { return label; }
  public Node getStartNode() { return startNode; }
  public Node getEndNode() { return endNode; }

  public void setLabel(String newLabel) { label = newLabel; }
  public void setStartNode(Node aNode) { startNode = aNode; }
  public void setEndNode(Node aNode) { endNode = aNode; }

  public boolean isSelected() { return selected; }
  public void setSelected(boolean state) { selected = state; }
  public void toggleSelected() { selected = !selected; }

  // Get the length of this edge
  public float length() {
    return (float)startNode.getLocation().distance(endNode.getLocation());
  }

  // Get the edge's Node at the other end from the specified one
  public Node otherEndFrom(Node aNode) {
    if (startNode == aNode)
      return endNode;
    else
      return startNode;
  }

  // Edges look like this:  sNode(12,43) --> eNode(67,34)
  public String toString() {
    return(startNode.toString() + " --> " + endNode.toString());
  }

  
  // Return true if the Edge is equal another Node passed in
  public boolean equals(Object obj) {
    if (obj == null) return false;
    if (!(obj instanceof Edge)) return false;
    Edge e = (Edge)obj;
    return (((startNode == e.startNode) && (endNode == e.endNode)) || ((startNode == e.endNode) && (endNode == e.startNode)));
  }
  
  // Required for Comparable interface
  public int hashCode() { return startNode.hashCode() + endNode.hashCode(); }
}