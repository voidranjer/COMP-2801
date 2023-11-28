import java.util.Collections;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.awt.*;
import java.awt.geom.Point2D.Double;

public class AreaCoverage {
  public static final float  ROBOT_RADIUS = 6.8f;              // in cm
  public static final float  ROBOT_BORDER_TOLERANCE = 0.01f;   // in cm
  public static final float  ROBOT_DIAMETER = ROBOT_RADIUS*2;  // in cm

  private Point 	        start;            // Start point of robot
  private Point	            end;              // End point of robot
  private VectorMap         vmap;             // Needs to be loaded
  private Graph             gridGraph;        // Needs to be computed
  private ArrayList<java.awt.geom.Point2D.Double>  path;             // Path in the spanning tree
  private int               edgeCount;        // the number of new edges traversed on the path
  private Node              startNode;        // starting Node of the Spanning Tree
  
  // An Area Coverage will allow the robot to perform area coverage
  public AreaCoverage(VectorMap  vm) {
    start = null;  // Needs to be provided later
    end = null;    // Needs to be provided later
    vmap = vm;
    gridGraph = null;    // Needs to be computed
    path = new ArrayList<java.awt.geom.Point2D.Double>();
    startNode = null;
  }

  // These are used to refresh the start or end points or the map
  public void setStart(int x, int y) { start = new Point(x, y); }
  public void setEnd(int x, int y) { end = new Point(x,y); }
  
  public VectorMap getVectorMap() { return vmap; }
  public void setVectorMap(VectorMap m) { vmap = m; }
  public Graph getGridGraph() { return gridGraph; }


  // This function returns true if the given node would cause the robot
  // at that location to intersect with an obstacle or the environment bounary.
  public boolean isInValid(Node n, ArrayList<Obstacle> obstacles, double width, double height) {
    java.awt.geom.Point2D.Double p = n.getLocation();
      
    // First check if the node goes beyond the boundary and remove it if so
    if ((p.x < ROBOT_RADIUS) || (p.y < ROBOT_RADIUS) || (p.x > (width-ROBOT_RADIUS)) || (p.y > (height-ROBOT_RADIUS))) 
      return true;
      
    // Now check for intersection with obstacle
    for (Obstacle ob: vmap.getObstacles()) {
      if (ob.contains(p))
        return true;
      for (int i=0; i<ob.numVertices(); i++) {
        Point v1 = ob.getVertex(i);
        Point v2 = ob.getVertex((i+1)%ob.numVertices());
        if (java.awt.geom.Line2D.ptSegDist(v1.x, v1.y, v2.x, v2.y, p.x, p.y) <= ROBOT_RADIUS) 
            return true;
      }
    }
    return false;  
  }


  // This function computes the bisector nodes (i.e., extended vertices) for each corner of every obstacle
  public void computeExtendedVertices(ArrayList<Obstacle> obstacles) {
    // Go through each obstacle and find the bisector node locations
    for (Obstacle ob: obstacles) {
      ob.getExtendedVertices().clear();
      // Go through the edges of the obstacle
      for (int i = 0; i < ob.numVertices(); i++) {
        Point vi = ob.getVertex(i);
        Point vip1 = ob.getVertex((i + 1) % ob.numVertices());
        Point vim1 = ob.getVertex((i - 1 + ob.numVertices()) % ob.numVertices());
        Double p = new Double();

        double dot = (vip1.x - vi.x) * (vim1.x - vi.x) + (vip1.y - vi.y) * (vim1.y - vi.y);
        double magA = Math.sqrt((vip1.x - vi.x) * (vip1.x - vi.x) + (vip1.y - vi.y) * (vip1.y - vi.y));
        double magB = Math.sqrt((vim1.x - vi.x) * (vim1.x - vi.x) + (vim1.y - vi.y) * (vim1.y - vi.y));
        double phi = Math.acos(dot / magA / magB);
        double d = (ROBOT_RADIUS + ROBOT_BORDER_TOLERANCE) / Math.sin(phi / 2);
        double sigma = Math.atan2((vip1.y - vi.y), (vip1.x - vi.x));
        double theta = sigma + phi / 2 + Math.PI;
        p.x = vi.x + d * Math.cos(theta);
        p.y = vi.y + d * Math.sin(theta);

        ob.addExtendedVertex(p);
      }
    }
  }



  // This method adds Nodes along the obstacle edges and connects them in the gridGraph
  public void addNodesAlongEdges(ArrayList<Obstacle> obstacles) {

    // ADD YOUR CODE HERE
    for (Obstacle ob : obstacles) {
      for (int i = 0; i < ob.getExtendedVertices().size(); i++) {
        java.awt.geom.Point2D.Double vi, vip1;
        vi = ob.getExtendedVertex(i);
        vip1 = ob.getExtendedVertex((i+1)%ob.numVertices());
        
        double dist = vi.distance(vip1);
        int nCount = (int) Math.ceil(dist/ROBOT_DIAMETER - 1);
        double gap = dist/(nCount+1);

        double xs = vi.x;
        double xe = vip1.x;
        double ys = vi.y;
        double ye = vip1.y;

        gridGraph.addNode(new Node(vi));
        for (int k = 1; k <= nCount; k++) {
          double xk = xs + (xe-xs)*gap/dist*k;
          double yk = ys + (ye-ys)*gap/dist*k;
          
          Node node = new Node(new java.awt.geom.Point2D.Double(xk, yk));
          gridGraph.addNode(node);
        }
        // gridGraph.addNode(new Node(vip1));
      }
    }
    // for (Obstacle obstacle : obstacles) {
    //   for (java.awt.geom.Point2D.Double vertex: obstacle.getExtendedVertices()) {
    //     Node node = new Node(vertex);
    //     gridGraph.addNode(node);
    //   }
    // }

  }



  // Find and return all Nodes that intersect an obstacle or are out of range
  // and select all nodes that are the beginning of a connected component
  public ArrayList<Node> findBadNodes(ArrayList<Obstacle> obstacles, double width, double height) {
    ArrayList<Node> badNodes = new ArrayList<Node>();
    
    // ADD YOUR CODE HERE
    
    return badNodes;
  }



  // Create the nodes/edges along the obstacle borders and add them to the spanning tree
  public void addBorderingNodesAndEdges(ArrayList<Obstacle> obstacles, double width, double height) {
    // STEP 1 - Go through each obstacle and find the bisector node locations
    computeExtendedVertices(obstacles);

    // STEP 2 - Now add Nodes along the obstacle edges and connect them in the gridGraph
    addNodesAlongEdges(obstacles);

    // STEP 3 - Find and return all Nodes that intersect an obstacle or are out of range
    //          and select all nodes that are the beginning of a connected component
    ArrayList<Node> badNodes = findBadNodes(obstacles, width, height);

    // Remove those bad nodes
    for (Node n: badNodes)
      gridGraph.deleteNode(n); 
  }
  
  
  // Add the grid nodes to the graph
  private void addGridNodesAndEdges(ArrayList<Obstacle> obstacles, double width, double height) {
    // Make a grid of rows and columns
    int numRows = (int)(height/ROBOT_DIAMETER);
    int numCols = (int)(width/ROBOT_DIAMETER);
    
    // Create all the nodes
    Node  nodes[][] = new Node[numRows][numCols];
    for (int r=0; r<numRows; r++) {
      for (int c=0; c<numCols; c++) {
        nodes[r][c] = new Node(new java.awt.geom.Point2D.Double((int)(c*ROBOT_DIAMETER + ROBOT_RADIUS), (int)(r*ROBOT_DIAMETER + ROBOT_RADIUS)));
        gridGraph.addNode(nodes[r][c]);
      }
    }
    
    // Now connect them with edges;
    for (int r=0; r<numRows; r++) {
      for (int c=0; c<numCols-1; c++) {
        gridGraph.addEdge(nodes[r][c], nodes[r][c+1]);
      }
    }
    for (int c=0; c<numCols; c++) {
      for (int r=0; r<numRows-1; r++) {
        gridGraph.addEdge(nodes[r][c], nodes[r+1][c]);
      }
    }

    // Now find all Nodes that intersect an obstacle
    ArrayList<Node> badNodes = new ArrayList<Node>();
    OUTER:
    for (Node n: gridGraph.getNodes()) {
      java.awt.geom.Point2D.Double p = n.getLocation();
      for (Obstacle ob: vmap.getObstacles()) {
        if (ob.contains(p)) {
          badNodes.add(n);
          continue OUTER;
        }
        for (int i=0; i<ob.numVertices(); i++) {
          Point v1 = ob.getVertex(i);
          Point v2 = ob.getVertex((i+1)%ob.numVertices());
          
          if (java.awt.geom.Line2D.ptSegDist(v1.x, v1.y, v2.x, v2.y, p.x, p.y) <= ROBOT_RADIUS) 
            badNodes.add(n);
        }
      }
    }
    // Remove those bad nodes
    for (Node n: badNodes)
      gridGraph.deleteNode(n);   
  }
  
  
  // Create and store a new grid graph for the set of convex obstacles
  public void computeGridGraph() {
    ArrayList<Obstacle> obstacles = vmap.getObstacles();   // environment obstacles
    double width = vmap.getWidth()*vmap.getResolution();   // environment width in cm
    double height = vmap.getHeight()*vmap.getResolution(); // environment height in cm
    
    // Create and store the graph for display access later
    gridGraph = new Graph();

    if (MapperApp.ShowGridNodes)
      addGridNodesAndEdges(obstacles, width, height);
      
    // Find the Node closest to (x,y).  This will be used for the start of the spanning tree
    if (gridGraph.getNodes().size() > 0) {
      startNode = gridGraph.getNodes().get(0);
      for (Node n: gridGraph.getNodes()) {
        if (n.getLocation().distanceSq(start.x, start.y) < startNode.getLocation().distanceSq(start.x, start.y))
          startNode = n;
      }
    }
    else startNode = new Node(new Point(0,0)); // Only to prevent error when building up code for the lab
    
    if (MapperApp.ShowBorderNodes)
      addBorderingNodesAndEdges(obstacles, width, height);
    
    System.out.println(gridGraph);
  }


  // Recursively compute the path in the spanning tree from the starting location having come in from edge e
  private void computeCoveragePathFrom(Node n, Edge e) {
    // If previous pointer has been set, then we were here already
    if (n.getPrevious() != null) 
      return;
      
    //Add this node to the path
    path.add(n.getLocation());
    
    n.setPrevious(e.otherEndFrom(n));
    if (path.size() > 1)
      edgeCount--; 
    
    ArrayList<Node> neighbours = n.neighbours();
    //Collections.sort(neighbours);
    for (Node neigh: neighbours) 
      computeCoveragePathFrom(neigh, n.getNeighbouringEdgeTo(neigh));
    
    // Add the node to the path on the way back from the recursion
    if (edgeCount > 0) 
      path.add(n.getPrevious().getLocation());
  }

  // Recursively compute the heights of ther nodes in the spanning tree from the starting root node
  private void computeNodeHeightsFrom(Node n) {
    n.setVisited(true);
    	
    ArrayList<Edge> edges = n.incidentEdges();
    if (edges.size() == 1) 
      n.setDistance(0);
    else
      n.setDistance(1);
      
    int max = 0;
    for (Edge e: edges) {
      Node n2 = e.otherEndFrom(n);
      if (!n2.hasBeenVisited()) {
		computeNodeHeightsFrom(n2);
      	if (n2.getDistance() > max)
        	max = (int)n2.getDistance();
      }
    }
    n.setDistance(n.getDistance() + max);
  }

  
  // Recursively compute the depth-first order spanning tree from the grid graph starting at node n, coming from edge e
  private void computeSpanningTreeFrom(Node n, Edge e) {
    // If previous pointer has been set, then we were here already
    if (n.hasBeenVisited()) 
      return;
    
    // Mark the node and the edge as visited
    n.setVisited(true);
    e.setSelected(true);
    
    ArrayList<Edge> edges = n.incidentEdges();
    for (Edge ed: edges)
      computeSpanningTreeFrom(ed.otherEndFrom(n), ed);
  }


  // Compute the depth-first order spanning tree from the grid graph with the starting node closest to the start point
  public void computeSpanningTree() {    
    // If there are no nodes, then stop now
    if ((gridGraph == null) || (gridGraph.getNodes().isEmpty()))
      return;
      
    // Reset all the previous pointers
    for (Node n: gridGraph.getNodes()) {
      n.setVisited(false);
      n.setDistance(0);
    }
    
    // Recursively compute the spanning tree
    // We pass in a "dummy edge" that is not part of the tree just so that we don't have to check for null
    Edge dummyEdge = new Edge(startNode, startNode);
    computeSpanningTreeFrom(startNode, dummyEdge);
   
    // Now remove all edges that are not selected
    ArrayList<Edge>  allEdges = gridGraph.getEdges();
    for (Edge e: allEdges)
      if (!e.isSelected())
        gridGraph.deleteEdge(e);
        
    // Now add the border and obstacle-coverage nodes/edges to the spanning tree if neeeded
    if (MapperApp.ShowGridNodes && MapperApp.ShowBorderNodes) {
      for (Node sn: gridGraph.selectedNodes()) {
        // Find the closes node to the selected node and connect it
        Node closest = gridGraph.getNodes().get(0);
        for (Node n: gridGraph.getNodes()) {
          // Ignore nodes at the same location
          if ((sn.getLocation().x == n.getLocation().x) && (sn.getLocation().y == n.getLocation().y))
            continue;
          if (n.getLocation().distanceSq(sn.getLocation().x, sn.getLocation().y) < closest.getLocation().distanceSq(sn.getLocation().x, sn.getLocation().y))
            closest = n;
        }
        // Now add an edge in the spanning tree from the closest node to the selected node
        gridGraph.addEdge(closest, sn);
      }
    }      
        
    // Now compute the node heights
    for (Node n: gridGraph.getNodes()) { 
      n.setVisited(false);
      n.setDistance(0);
    }
    computeNodeHeightsFrom(startNode);
    
    // Prepare all node labels
    //for (Node n: gridGraph.getNodes()) n.setLabel(""+(int)n.getDistance());

    // Now compute the coverage path
    path.clear();                        // Clear the previously-computed path, if there was one
    edgeCount = gridGraph.getEdges().size();
    for (Node n: gridGraph.getNodes())  // Reset all the previous pointers
      n.setPrevious(null);
    // We pass in a "dummy edge" that is not part of the tree just so that we don't have to check for null
    computeCoveragePathFrom(startNode, dummyEdge);
  }

  // Get the covergae path as a set of points
  public ArrayList<java.awt.geom.Point2D.Double> getCoveragePath() {
    return path;
  }
}