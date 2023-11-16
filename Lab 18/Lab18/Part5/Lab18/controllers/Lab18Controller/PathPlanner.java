import java.util.ArrayList;
import java.util.PriorityQueue;
import java.awt.*;

public class PathPlanner {
  private static final Color   OBSTACLE_COLOR = new Color(150, 200, 255);       // light pale blue
  private static final Color   GROWN_OBSTACLE_COLOR = new Color(200, 220, 255); // light pale blue
  private static final Color   OBSTACLE_ID_COLOR = new Color(150, 0, 0);        // Dark red

  private static final float  ROBOT_RADIUS = 6.8f;       // in cm
  private static final int    CORNER_DEGREE_UNIT = 30; // in degrees

  private Point 	      start;            // Start point of robot
  private Point	      end;              // End point of robot
  private Point             originalEnd;      // Copy of the original end point (since mouse position may update it)
  private Node              previousEndNode;  // Need to remove this from graph before updating on mouse move
  private VectorMap         vmap;             // Needs to be loaded
  private ArrayList<Point>  supportLines;     // Needs to be computed.  Support lines from start location only
  private Graph             visibilityGraph;  // Needs to be computed
  private boolean           graphIsForGrownObstacles; // True if and only if grown obstacles were used in the current visibility graph
  
  public static boolean     ShowObstacles = true;
  public static boolean     ShowConvexObstacles = false;
  public static boolean     ShowGrownObstacles = true;
  public static boolean     ShowStartDest = true;
  public static boolean     ShowSupportLines = false;
  public static boolean     ShowOriginalGraph = false;
  public static boolean     ShowGrownGraph = true;
  public static boolean     ShowObstacleLabels = false;
  public static boolean     ShowPath = false;
  public static boolean     ShowDynamicPath = false;
  public static boolean     ShowTree = false;
  
  // A Path Planner also requires a starting and end location, but these will be set later
  public PathPlanner(VectorMap  vm) {
    start = null;  // Needs to be provided later
    end = null;    // Needs to be provided later
    vmap = vm;
    supportLines = null;    //Needs to be computed
    visibilityGraph = null; // Needs to be computed
    graphIsForGrownObstacles = false;
    previousEndNode = null; // Needs to be set later
  }

  // These are used to refresh the start or end points or the map
  public void setStart(int x, int y) { start = new Point(x, y); }
  public void setEnd(int x, int y) { 
    if (end == null)
      originalEnd = new Point(x, y);
    end = new Point(x,y); 
  }
  public void resetEndpoint() { 
    if (visibilityGraph != null)
      previousEndNode = visibilityGraph.node(end.x, end.y);
    end = originalEnd; 
  }
  public void setVectorMap(VectorMap m) { vmap = m; }
  public Graph getVisibilityGraph() { return visibilityGraph; }


  // Returns true if a support line from (x,y) to (supportX, supportY) intersects an obstacle
  // at a point other than at a vertex.
  public boolean supportLineIntersectsObstacle(int x, int y, int supportX, int supportY, Obstacle fromObst, ArrayList<Obstacle> obstacles) {
    // First, handle case where (x,y) or (supportX, SupportY) is a vertex of the original obstacle.
    if (fromObst != null) {
      Obstacle oob = fromObst.getOriginalObstacle();
      boolean intersects = false;
      for (int i=0; i<oob.numVertices(); i++) {
        Point v1 = oob.getVertex(i);
        Point v2 = oob.getVertex((i+1)%oob.numVertices());
        boolean xyIsStartOfEdge = (x == v1.x) && (y == v1.y);
        boolean xyIsEndOfEdge = (x == v2.x) && (y == v2.y);
        boolean sxsyIsStartOfEdge = (supportX == v1.x) && (supportY == v1.y);
        boolean sxsyIsEndOfEdge = (supportX == v2.x) && (supportY == v2.y);
        // If it is an edge of the original obstacle, then this is ok 
        if ((xyIsStartOfEdge && sxsyIsEndOfEdge) || (xyIsEndOfEdge && sxsyIsStartOfEdge)) {
          return false;
        }
        // Check if the midpoint of (x,y)-->PR is inside the original polygon.
        // This will remove the shared edges that are the result of a triangulation.
        Point mid = new Point((x+supportX)/2, (y+supportY)/2);                                                                                                               
        if (oob.contains(mid))
          intersects = true; 
        
        // Check if the support line now intersects the original obstacle 
        if (!xyIsStartOfEdge && !xyIsEndOfEdge && !sxsyIsStartOfEdge && !sxsyIsEndOfEdge)
          if (java.awt.geom.Line2D.Double.linesIntersect(x, y, supportX, supportY, v1.x, v1.y, v2.x, v2.y)) 
            intersects = true;
      }
      if (intersects) return true;
    }
    // Now check it against all other obstacles that were not from the same original obstacle
    for (Obstacle ob: obstacles) {
      if ((fromObst != null) && (fromObst.getOriginalObstacle() == ob.getOriginalObstacle())) {
        // Check if the distance from a vertex of the original polygon to this support point is too small
        if (ob != ob.getOriginalObstacle()) {
          for (Point p: ob.getOriginalObstacle().getVertices())
            if (!((x == p.x) && (y == p.y)) && !((supportX == p.x) && (supportY == p.y))) {
              double d = 0;
              if ((d = java.awt.geom.Line2D.ptSegDist(x, y, supportX, supportY, p.x, p.y)) < ROBOT_RADIUS) {
                //System.out.println("Bad Support: ("+x+","+y+") -- ("+supportX+","+supportY+") too close to ("+p.x+","+p.y+") distance " + d);
                return true;
              }
            }
        }
        continue;
      }
      for (int i=0; i<ob.numVertices(); i++) {
        Point v1 = ob.getVertex(i);
        Point v2 = ob.getVertex((i+1)%ob.numVertices());
        
        // Make sure that the (x,y) and (supportX, supportY) are not vertices of the edge being checked
        // This happens when two triangles share the same boundary
        boolean xyIsStartOfEdge = (x == v1.x) && (y == v1.y);
        boolean xyIsEndOfEdge = (x == v2.x) && (y == v2.y);
        boolean sxsyIsStartOfEdge = (supportX == v1.x) && (supportY == v1.y);
        boolean sxsyIsEndOfEdge = (supportX == v2.x) && (supportY == v2.y);
        
        // Since it is not attached to this edge being checked, make sure there is no intersection
        if (!xyIsStartOfEdge && !xyIsEndOfEdge && !sxsyIsStartOfEdge && !sxsyIsEndOfEdge)
          if (java.awt.geom.Line2D.Double.linesIntersect(x, y, supportX, supportY, v1.x, v1.y, v2.x, v2.y)) 
            return true;
      }
    }
    return false; // Since there was no intersection
  }


  // Compute and return a list of the Visible Support Points with respect to the given point
  public ArrayList<Point> computeSupportPointsFrom(ArrayList<Obstacle> obstacles, Obstacle fromObst, int x, int y) {
    ArrayList<Point>      supports = new ArrayList<Point>();
    Point                 intersection = null;

    // Go through the polygons and find the support points
    for (Obstacle ob: obstacles) {
      Point  PL = null, PR = null;  // Assume no supports to start

      for (int i=0; i<ob.numVertices(); i++) {
        Point  vi = ob.getVertex(i);
        Point bvi = ob.getVertex((i-1+ob.numVertices())%ob.numVertices());
        Point avi = ob.getVertex((i+1)%ob.numVertices());
        
        // Check if we are looking for the support lines from ob's vertex
        // If so, just add one ... we will get the other one from a different vantage point
        if ((x == vi.x) && (y == vi.y)) {
          PL = new Point(bvi.x, bvi.y);
          if (!supportLineIntersectsObstacle(x, y, PL.x, PL.y, ob, obstacles))
          supports.add(PL);
          continue;
        }
        // Calculate the turn types
        double t1 = (vi.x-x)*(avi.y-y) - (vi.y-y)*(avi.x-x);
        double t2 = (vi.x-x)*(bvi.y-y) - (vi.y-y)*(bvi.x-x);	
				
        if ((t1 <= 0) && (t2 <= 0)) {
          PL = new Point(vi.x, vi.y);
          if (!supportLineIntersectsObstacle(x, y, PL.x, PL.y, ob, obstacles)) 
            supports.add(PL);
        }
        if ((t1 >= 0) && (t2 >= 0)) {
          PR = new Point(vi.x, vi.y);
          if (!supportLineIntersectsObstacle(x, y, PR.x, PR.y, ob, obstacles))
            supports.add(PR);
        }
      }
    }

    // Now add a line from the start to the destination if it does not intersect any polygons
    Point END = new Point(end.x, end.y);
    if (!supportLineIntersectsObstacle(x, y, END.x, END.y, null, obstacles))
      supports.add(END);
      
    return supports;
  }



  // Update the visibility graph by rermoving all edges frrom the old end location and adding ones to the new end location
  public void updateVisibilityGraph() {
    ArrayList<Point>    supports;

    if (visibilityGraph == null) {
      computeVisibilityGraph();
      return;
    }
    ArrayList<Obstacle> obstacles;
    if (ShowGrownObstacles || ShowGrownGraph)
      obstacles = vmap.getGrownObstacles();
    else
      obstacles = vmap.getConvexObstacles();
    
    // Remove the previous End point
    visibilityGraph.deleteNode(previousEndNode);
    // Make the new end node.
    Node e = new Node("", end);
    previousEndNode = e; // remember it for updating later
    visibilityGraph.addNode(e);

    // Now add a line from the start to the destination if it does not intersect any polygons
    Point END = new Point(end.x, end.y);
    if (!supportLineIntersectsObstacle(start.x, start.y, END.x, END.y, null, obstacles))
      visibilityGraph.addEdge(visibilityGraph.node(start.x, start.y), e);

    
    // Compute the support lines from all graph nodes corresponding to obstacle vertices
    for (Node n: visibilityGraph.getNodes()) {
      if (!supportLineIntersectsObstacle(n.getLocation().x, n.getLocation().y, END.x, END.y, null, obstacles))
        visibilityGraph.addEdge(n, e);
    }
  }


  // Create and store a new visibility graph for the set of convex obstacles
  public void computeVisibilityGraph() {
    ArrayList<Obstacle> obstacles;

    // Choose the corect obstacles to be using based on the GUI settings
    graphIsForGrownObstacles = false;
    if (ShowGrownObstacles || ShowGrownGraph) {
      obstacles = vmap.getGrownObstacles();
      graphIsForGrownObstacles = true;
    }
    else
      obstacles = vmap.getConvexObstacles();


    ArrayList<Point>    supports;
    
    // Create and store the graph for display access later
    visibilityGraph = new Graph();

    // Add the start & end points as graph nodes
    Node s = new Node(start);
    visibilityGraph.addNode(s);
    Node e = new Node(end);
    previousEndNode = e; // remember it for updating later
    visibilityGraph.addNode(e);

    // Add all the obstacle vertices as graph nodes
    for (Obstacle ob: obstacles) {
      for (int i=0; i<ob.numVertices(); i++) {
        Point p = new Point(ob.getVertex(i).x, ob.getVertex(i).y);
        // Check to make sure that the vertex is not a duplicate
        if (visibilityGraph.node(p.x, p.y) == null) {
          // Now make sure that it does not lie within another obstacle
          boolean bad = false;
          for (Obstacle ob2: obstacles) {
            if ((ob != ob2) && ob2.contains(p) && !(ob2.pointOnBoundary(p)))
              bad = true;
          }
          if (!bad)
            visibilityGraph.addNode(new Node(p, ob));
        }
      }
    }

    // Compute the support lines from the start location
    supports = computeSupportPointsFrom(obstacles, null, start.x, start.y);
    supportLines = supports;
    for (Point p: supports) {
      Node m = visibilityGraph.node(p.x, p.y);
      if (m != null)
        visibilityGraph.addEdge(s, m);
    }
    
    // Compute the support lines from all graph nodes corresponding to obstacle vertices
    for (Node n: visibilityGraph.getNodes()) {
      if ((n == s) || (n==e))
        continue;
      supports = computeSupportPointsFrom(obstacles, n.getObstacle(), n.getLocation().x, n.getLocation().y);
      for (Point p: supports) {
        Node m = visibilityGraph.node(p.x, p.y);
        if (m != null)
          visibilityGraph.addEdge(n, m);
      }
    }
    
    // Remove all edges that are too close to a vertex due to rounding errors
    if (graphIsForGrownObstacles) {
      ArrayList<Edge> badEdges = new ArrayList<Edge>();
      for (Edge edge: visibilityGraph.getEdges()) {
        for (Obstacle ob: vmap.getObstacles()) {
          for (Point p: ob.getVertices()) {
            Point e1 = edge.getStartNode().getLocation();
            Point e2 = edge.getEndNode().getLocation();
            double d=0;
            if ((d = java.awt.geom.Line2D.ptSegDist(e1.x, e1.y, e2.x, e2.y, p.x, p.y)) < ROBOT_RADIUS) {
              //System.out.println("Bad Edge: ("+e1.x+","+e1.y+") -- ("+e2.x+","+e2.y+") too close to ("+p.x+","+p.y+") distance " + d);
              badEdges.add(edge);
            }
          }
        }
      }
      for (Edge edge: badEdges)
        visibilityGraph.deleteEdge(edge);
    }
  }

  
  // This code runs Dijkstra's shortest path algorithm in the visibilty graph
  private void runDijkstrasAlgorithmFrom(Node s) {
    // Set up a priority queue with all nodes
    PriorityQueue<Node> queue = new PriorityQueue<Node>();
        
    // Initialize with infinite cost to all nodes, except the start node
    for(Node n: visibilityGraph.getNodes()) {
      n.setDistance(999999);
      n.setPrevious(null);
      queue.add(n);
    }
    s.setDistance(0);
        
    // Now find the path by updating distances to all nodes and storing their previous nodes.
    while(!queue.isEmpty()) {
      Node min = queue.remove();
      //if (min == e) break;
      for(Edge ed: min.incidentEdges()) {
        Node n = ed.otherEndFrom(min);
        if (n.getDistance() > (min.getDistance() + ed.length())) {
          n.setDistance(min.getDistance() + ed.length());
          n.setPrevious(min);
          queue.remove(n);
          queue.add(n);
        }
      }
    }
  }
  
  // Compute the shortest path tree in the visibility graph (assumed computed).  Highlight the 
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
    e.setLabel("("+end.x+","+end.y+")");
    s.setLabel("("+start.x+","+start.y+")");
    
    // Compute the shortest path tree
    runDijkstrasAlgorithmFrom(s);

    // Unselect all the nodes and edges
    for(Node n: visibilityGraph.getNodes())
      n.setSelected(false);
    for(Edge ed: visibilityGraph.getEdges())
      ed.setSelected(false);
        
    // Now trace the path back from the end node and select all edges and Nodes along the path
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
    }
    else if (ShowTree) {
      for(Node n: visibilityGraph.getNodes()) {
        Edge ed = visibilityGraph.edge(n, n.getPrevious());
        if (ed != null)
          ed.setSelected(true);
      }
      s.setSelected(true);
    }
  }
  
  
  // Assuming that the shortest path has been computed already, get the shortest
  // path from the start to the end as a set of points in the environment.
  // Assuming that the shortest path has been computed already, get the shortest
  // path from the start to the end as a set of points in the environment.
  public ArrayList<Point> getShortestPath() {
    ArrayList<Point>  sp = new ArrayList<Point>();  // Do NOT remove this
    
    // Handle case where graph was not yet computed
    if (visibilityGraph == null) {  // Do NOT remove this IF statement
      sp.add(start);
      return sp;
    }
    
    
    // ADD YOUR CODE HERE

    
    return sp;                                     // Do NOT remove this
  } 
  
  
  
  // Grow the obstacles according to the size of the robot
  public void growObstacles() {
    vmap.getGrownObstacles().clear();
    for (Obstacle ob: vmap.getConvexObstacles()) 
      vmap.getGrownObstacles().add(ob.grow(ROBOT_RADIUS, CORNER_DEGREE_UNIT));
  }
}