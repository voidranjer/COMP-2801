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
  
  public static boolean     ShowObstacles = true;
  public static boolean     ShowPositions = false;
  public static boolean     ShowGridGraph = false;
  public static boolean     ShowSpanningTree = false;

  // This panel draws everything and contains the map
  private MapperPanel  mapPanel;
  
  // This is the area coverage that computes the are coverage grid graph and spanning tree from the vector map in the mapPanel
  private AreaCoverage  areaCoverage;
  public AreaCoverage getAreaCoverage() { return areaCoverage; }
  
  public MapperApp(int widthInCentimeters, int heighInCentimeters) {
    super("Mapper App");
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification)+ MARGIN_X*2;
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification) + MARGIN_Y*2;
    
    add(mapPanel = new MapperPanel((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution, Magnification));

    // Start with an area coverage based on an empty vector map to start
    areaCoverage = new AreaCoverage(mapPanel.getMap().getVectorMap()); 
    mapPanel.setAreaCoverage(areaCoverage);

    // Create the menu bar and menus
    JMenuBar menuBar = new JMenuBar();
    setJMenuBar(menuBar);
    
    // Set up the Area Coverage Menu
    JMenu	  acMenu = new JMenu("AreaCoverage"); menuBar.add(acMenu);
    JCheckBoxMenuItem showPosGridButton = new JCheckBoxMenuItem("Show Position Grid");  acMenu.add(showPosGridButton); showPosGridButton.setSelected(false); acMenu.add(new JSeparator());
    JRadioButtonMenuItem hideGridGraphButton = new JRadioButtonMenuItem("Hide Graph");  acMenu.add(hideGridGraphButton); hideGridGraphButton.setSelected(true);
    JRadioButtonMenuItem showGridGraphButton = new JRadioButtonMenuItem("Show Grid Graph");  acMenu.add(showGridGraphButton); showGridGraphButton.setSelected(false);
    JRadioButtonMenuItem showSpanningTreeButton = new JRadioButtonMenuItem("Show Spanning Tree");  acMenu.add(showSpanningTreeButton); acMenu.add(new JSeparator());
    ButtonGroup group = new ButtonGroup();
    group.add(hideGridGraphButton); group.add(showGridGraphButton); group.add(showSpanningTreeButton); 
       
    // Event Handlers for Area Coverage algorithm
    hideGridGraphButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        ShowGridGraph = false;
        ShowSpanningTree = false;
        mapPanel.update();
      }});
    showPosGridButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        ShowPositions = showPosGridButton.isSelected();
        areaCoverage.computeGridGraph();
        areaCoverage.computeSpanningTree();
        mapPanel.update();
      }});
    showGridGraphButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        ShowGridGraph = true;
        ShowSpanningTree = false;
        areaCoverage.computeGridGraph();
        mapPanel.update();
      }});
    showSpanningTreeButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        ShowGridGraph = false;
        ShowSpanningTree = true;
        areaCoverage.computeGridGraph();
        areaCoverage.computeSpanningTree();
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
 	mapper.getAreaCoverage().setStart(15, 150); // This will be the start/end for our testing
	mapper.getAreaCoverage().getVectorMap().loadMapFromWorld();
  }
}
