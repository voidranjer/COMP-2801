import java.io.*;
import java.util.ArrayList;
import java.awt.Graphics;
import java.awt.Point;

public class Graph {
  private ArrayList<Node>    nodes;    // We just keep the Nodes in a list, each Node keeps the edges

  // Some constructors
  public Graph() { this(new ArrayList<Node>()); }

  public Graph(ArrayList<Node> initialNodes) {
    nodes = initialNodes;
  }

  // The get & set methods
  public ArrayList<Node> getNodes() { return nodes; }

  // Get all the edges of the graph by asking the nodes for them
  public ArrayList<Edge> getEdges() {
    ArrayList<Edge>  edges = new ArrayList<Edge>();
    for (Node n: nodes) {
      for (Edge e: n.incidentEdges()) {
        if (!edges.contains(e)) //so that it is not added twice
          edges.add(e);
      }
    }
    return edges;
  }

  // Graphs look like this:  label(6 nodes, 15 edges)
  public String toString() {
    return("Graph: (" + nodes.size() + " nodes, " + getEdges().size() + " edges)");
  }

  public void addNode(Node aNode) { nodes.add(aNode); }

  // Add an Edge to the graph, given the start and end Nodes
  public Edge addEdge(Node start, Node end) {
    // First, make sure the edge is not already there (for undirected graph)
    boolean thereAlready = false;
    ArrayList<Edge> edges = start.incidentEdges();
    for (Edge e: edges) {
      if (e.otherEndFrom(start) == end)
        thereAlready = true;
    }
    edges = end.incidentEdges();
    for (Edge e: edges) {
      if (e.otherEndFrom(end) == start)
        thereAlready = true;
    }
    if (!thereAlready) {
      // First make the edge
      Edge anEdge = new Edge(start, end);
      // Now tell the nodes about the edge
      start.addIncidentEdge(anEdge);
      end.addIncidentEdge(anEdge);
      return anEdge;
    }
    return null;
  }

  public void addEdge(String startLabel, String endLabel) {
    Node  start = nodeNamed(startLabel);
    Node  end = nodeNamed(endLabel);
    if ((start != null) && (end != null))
      addEdge(start, end);
  }

  public void deleteEdge(Edge anEdge) {
    // Just ask the nodes to remove it from their list
    anEdge.getStartNode().incidentEdges().remove(anEdge);
    anEdge.getEndNode().incidentEdges().remove(anEdge);
  }

  public void deleteNode(Node aNode) {
    // Remove the opposite node's incident edges
    for (Edge e:   aNode.incidentEdges())
      e.otherEndFrom(aNode).incidentEdges().remove(e);
    nodes.remove(aNode);  // Remove the node now
  }

  public Node nodeNamed(String aLabel) {
    for (Node n:  nodes)
      if (n.getLabel().equals(aLabel)) return n;
    return null;  // If we don't find one
  }

  // Return the first Node in which point p is contained, if none, return null
  public Node nodeAt(Point p) {
    for (Node n:  nodes) {
      java.awt.geom.Point2D.Double c = n.getLocation();
      double d = (p.x - c.x) * (p.x - c.x) + (p.y - c.y) * (p.y - c.y);
      if (d <= (Node.RADIUS * Node.RADIUS))  return n;
    }
    return null;
  }

  // Return the Node with the given x and y point
  public Node node(int x, int y) {
    for (Node n:  nodes) {
      if ((n.getLocation().x == x) && (n.getLocation().y == y))
        return n;
    }
    return null;
  }

  // Return the Edge with the given Nodes
  public Edge edge(Node s, Node e) {
    for (Edge ed:  getEdges()) {
      if (((ed.getStartNode() == s) && (ed.getEndNode() == e)) || ((ed.getStartNode() == e) && (ed.getEndNode() == s)))
        return ed;
    }
    return null;
  }

  // Return the first Edge in which point p is near the midpoint; if none, return null
  public Edge edgeAt(Point p) {
    double      midPointX, midPointY;

    for (Edge e:  getEdges()) {
      midPointX = (e.getStartNode().getLocation().x + e.getEndNode().getLocation().x) / 2;
      midPointY = (e.getStartNode().getLocation().y + e.getEndNode().getLocation().y) / 2;
      double distance = (p.x - midPointX) * (p.x - midPointX) + (p.y - midPointY) * (p.y - midPointY);
      if (distance <= (Node.RADIUS * Node.RADIUS))
        return e;
    }
    return null;
  }

  // Get all the Nodes that are selected
  public ArrayList<Node> selectedNodes() {
    ArrayList<Node>   selected = new ArrayList<Node>();
    for (Node n:  nodes)
      if (n.isSelected()) selected.add(n);
        return selected;
  }

  // Get all the Edges that are selected
  public ArrayList<Edge> selectedEdges() {
    ArrayList<Edge>   selected = new ArrayList<Edge>();
    for (Edge e:  getEdges())
      if (e.isSelected()) selected.add(e);
    return selected;
  }

  // Draw the Graph at the specified magnification factor with the given Pen on a panel with the given height
  public void draw(Graphics aPen, int height, int magnification) {
    ArrayList<Edge>    edges = getEdges();

    for (Edge e: edges) {  // Draw the edges first
      if (e.isSelected())
        e.draw(aPen, height, magnification, 5);
      else
        e.draw(aPen, height, magnification, 5);
    }
    for (Node n: nodes)   // Draw the nodes second
      n.draw(aPen, height, magnification);
  }

  // Draw the Graph (showing selected edges only) at the specified magnification factor with the given Pen on a panel with the given height
  public void drawSelectedOnly(Graphics aPen, int height, int magnification, int thickness) {
    for (Edge e: selectedEdges())  // Draw the edges first
      e.draw(aPen, height, magnification, thickness);
    for (Node n: selectedNodes())   // Draw the nodes second
      n.draw(aPen, height, magnification);
  }
}