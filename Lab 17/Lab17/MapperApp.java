import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.io.*;
import java.awt.Dimension;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

public class MapperApp extends JFrame {
  private static final long serialVersionUID = 99999999;
  
  static final float Resolution = 0.5f; // 0.5 cm per grid cell
  static final int Magnification = 1;   // 2x magnification
  static int WindowWidth;
  static int WindowHeight;
  static int MARGIN_X = 10;
  static int MARGIN_Y = 10;
  
  // This panel draws everything and contains the map
  private MapperPanel  mapPanel;
  
  // This is the path planner that computes the visibility graph from the vector map in the mapPanel
  private PathPlanner  pathPlanner;

  public PathPlanner getPathPlanner() { return pathPlanner; }
  
  
  public MapperApp(int widthInCentimeters, int heighInCentimeters) {
    super("Mapper App");
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification)+ MARGIN_X*2;
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification) + MARGIN_Y*2;
    
    add(mapPanel = new MapperPanel((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution, Magnification));

    // Start with a path planner based on an empty vector map to start
    pathPlanner = new PathPlanner(mapPanel.getMap().getVectorMap()); 
    mapPanel.setPathPlanner(pathPlanner);

    // Create the menu bar and menus
    JMenuBar menuBar = new JMenuBar();
    setJMenuBar(menuBar);
    
    // Set up the Line Path Planning Menu
    JMenu	  planningMenu = new JMenu("PathPlanning"); menuBar.add(planningMenu);
    JMenuItem  loadObstaclesButton = new JMenuItem("Load Obstacle Map");  planningMenu.add(loadObstaclesButton); planningMenu.add(new JSeparator());
    JCheckBoxMenuItem showObstaclesButton = new JCheckBoxMenuItem("Show Original Obstacles");  planningMenu.add(showObstaclesButton);
    JCheckBoxMenuItem showConvexObstaclesButton = new JCheckBoxMenuItem("Show Obstacles as Convex");  planningMenu.add(showConvexObstaclesButton);
    JCheckBoxMenuItem showObstacleLabelsButton = new JCheckBoxMenuItem("Show Obstacle Numbers");  planningMenu.add(showObstacleLabelsButton);
    JCheckBoxMenuItem showStartDestButton = new JCheckBoxMenuItem("Show Start/Destination");  planningMenu.add(showStartDestButton); planningMenu.add(new JSeparator());
    showStartDestButton.setSelected(true);
    ButtonGroup group = new ButtonGroup();
    JRadioButtonMenuItem hideGraphButton = new JRadioButtonMenuItem("Hide All Graph Computations");  planningMenu.add(hideGraphButton); group.add(hideGraphButton);
    hideGraphButton.setSelected(true);
    JRadioButtonMenuItem showSupportsButton = new JRadioButtonMenuItem("Show Support Lines From Mouse Location");  planningMenu.add(showSupportsButton); group.add(showSupportsButton);
    JRadioButtonMenuItem showGraphButton = new JRadioButtonMenuItem("Show Entire Visibility Graph");  planningMenu.add(showGraphButton); group.add(showGraphButton); planningMenu.add(new JSeparator());
    JMenuItem  graphInfoButton = new JMenuItem("Graph Information");  planningMenu.add(graphInfoButton);
    
    // Set up the Shortest Path Menu
    JMenu	  spMenu = new JMenu("ShortestPath"); menuBar.add(spMenu);
    JRadioButtonMenuItem hideSpButton = new JRadioButtonMenuItem("Hide Shortest Path");  spMenu.add(hideSpButton); hideSpButton.setSelected(true);
    JRadioButtonMenuItem showSpTreeButton = new JRadioButtonMenuItem("Show Shortest Path Tree");  spMenu.add(showSpTreeButton);
    JRadioButtonMenuItem showSpButton = new JRadioButtonMenuItem("Show Shortest Path");  spMenu.add(showSpButton);
    JRadioButtonMenuItem showSpMouseButton = new JRadioButtonMenuItem("Show Shortest Path To Mouse Location");  spMenu.add(showSpMouseButton);
    JRadioButtonMenuItem showSptMouseButton = new JRadioButtonMenuItem("Show Shortest TURN Path To Mouse Location");  spMenu.add(showSptMouseButton);
    group = new ButtonGroup();
    group.add(hideSpButton); group.add(showSpTreeButton); group.add(showSpButton); group.add(showSpMouseButton);group.add(showSptMouseButton);


    // Event Handlers for Path Planner
    loadObstaclesButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        mapPanel.getMap().getVectorMap().load();
        VectorMap.ShowVector = false;
        PathPlanner.ShowObstacles = true;
        showObstaclesButton.setSelected(true);
        PathPlanner.ShowConvexObstacles = false;
        showConvexObstaclesButton.setSelected(false);
        PathPlanner.ShowSupportLines = PathPlanner.ShowGraph = false;
        PathPlanner.ShowPath = PathPlanner.ShowDynamicPath = PathPlanner.MinimizeTurns = PathPlanner.ShowTree = false;
        hideGraphButton.setSelected(true);
        hideSpButton.setSelected(true);
        mapPanel.update();
      }});
    graphInfoButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        String info;
        if (pathPlanner.getVisibilityGraph() != null)
          info = "Graph has " + pathPlanner.getVisibilityGraph().getNodes().size() + " vertices and " + pathPlanner.getVisibilityGraph().getEdges().size() + " edges";
        else
          info = "You need to compute the graph first.  Select the \"Show Entire Visibility Graph\" button ";
        JOptionPane.showMessageDialog(null, info,"Information", JOptionPane.INFORMATION_MESSAGE);
        mapPanel.update();
      }});
    showObstaclesButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowObstacles = showObstaclesButton.isSelected();
        PathPlanner.ShowConvexObstacles = false;
        showConvexObstaclesButton.setSelected(false);
        mapPanel.update();
      }});
    showConvexObstaclesButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowConvexObstacles = showConvexObstaclesButton.isSelected();
        PathPlanner.ShowObstacles = false;
        showObstaclesButton.setSelected(false);
        mapPanel.update();
      }});
    showObstacleLabelsButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowObstacleLabels = showObstacleLabelsButton.isSelected();
        mapPanel.update();
      }});
    showStartDestButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowStartDest = showStartDestButton.isSelected();
        mapPanel.update();
      }});
    hideGraphButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowSupportLines = PathPlanner.ShowGraph = false;
        mapPanel.update();
      }});
    showSupportsButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowSupportLines = true;
        PathPlanner.ShowGraph = false;
        pathPlanner.computeVisibilityGraph();
        mapPanel.update();
      }});
    showGraphButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowGraph = true;
        PathPlanner.ShowSupportLines = false;
        pathPlanner.computeVisibilityGraph();
        mapPanel.update();
      }});


    hideSpButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowPath = PathPlanner.ShowTree = PathPlanner.ShowDynamicPath = false ;
        mapPanel.update();
      }});
    showSpTreeButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowTree = true;
        PathPlanner.ShowPath = PathPlanner.ShowDynamicPath = false;
        pathPlanner.computeVisibilityGraph();
        pathPlanner.computeShortestPath();
        mapPanel.update();
      }});
    showSpButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowPath = true;
        PathPlanner.ShowTree = PathPlanner.ShowDynamicPath = false;
        pathPlanner.resetEndpoint();
        pathPlanner.computeVisibilityGraph();
        pathPlanner.computeShortestPath();
        mapPanel.update();
      }});
    showSpMouseButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowDynamicPath = true;
        PathPlanner.MinimizeTurns = false;
        PathPlanner.ShowPath = PathPlanner.ShowTree = false;
        pathPlanner.computeVisibilityGraph();
        pathPlanner.computeShortestPath();
        mapPanel.update();
      }});
    showSptMouseButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        PathPlanner.ShowDynamicPath = true;
        PathPlanner.MinimizeTurns = true;
        PathPlanner.ShowPath = PathPlanner.ShowTree = false;
        pathPlanner.computeVisibilityGraph();
        pathPlanner.computeShortestPath();
        mapPanel.update();
      }});
      
    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setSize(WindowWidth+4, WindowHeight+52);
    setVisible(true);
  }

  // Get the vector map
  public VectorMap getVectorMap() {
    return mapPanel.getMap().getVectorMap();
  }
  
  public static void main(String[] args) {
    MapperApp   mapper = new MapperApp(400, 300); // Make sure these dimensions match the environment.  
    
    PathPlanner planner = mapper.getPathPlanner();
    planner.setStart(12, 150);
    planner.setEnd(388, 150);
  }
}
