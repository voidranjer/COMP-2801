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
  static final int Magnification = 2;  // 2x magnification
  static int WindowWidth;
  static int WindowHeight;
  static int MARGIN_X = 10;
  static int MARGIN_Y = 10;
  
  private MapperPanel  mapPanel;

  public MapperApp(int widthInCentimeters, int heighInCentimeters) {
    super("Mapper App");

    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification)+ MARGIN_X*2;
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification) + MARGIN_Y*2;
    
    add(mapPanel = new MapperPanel((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution, Magnification));


    // Create the menu bar and menus
    JMenuBar menuBar = new JMenuBar();
    setJMenuBar(menuBar);
    
    JMenu 	  mapMenu = new JMenu("GridMap"); menuBar.add(mapMenu);
    JMenuItem  loadGridMapButton = new JMenuItem("Load Grid Map");
    mapMenu.add(loadGridMapButton);
    loadGridMapButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map m = mapPanel.mapFromFile();
        WindowWidth = (int)(m.getWidth()/Resolution)+ MARGIN_X*2;
        WindowHeight = (int)(m.getHeight()/Resolution) + MARGIN_Y*2;
        setSize(WindowWidth+4, WindowHeight+52);
        recompute();
        mapPanel.update();
      }});

    JMenu	  thresholdMenu = new JMenu("BinaryThreshold"); menuBar.add(thresholdMenu);
    JCheckBoxMenuItem showBinaryButton = new JCheckBoxMenuItem("Show Binary Map");  thresholdMenu.add(showBinaryButton);
    SpinnerModel model = new SpinnerNumberModel(0, 0, 255, 1);
    JSpinner  binarySpinner = new JSpinner(model);  thresholdMenu.add(binarySpinner);
    JCheckBoxMenuItem showBordersButton = new JCheckBoxMenuItem("Show Borders");  thresholdMenu.add(showBordersButton);
    showBinaryButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowBinary = showBinaryButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    showBordersButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowBorders = showBordersButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    binarySpinner.addChangeListener(new ChangeListener() {
      public void stateChanged(ChangeEvent e) {
        Map.BINARY_THRESHOLD = (int)(binarySpinner.getValue());
        recompute();
        mapPanel.update();
      }});

    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setSize(WindowWidth+4, WindowHeight+52);
    setVisible(true);
  }
  
  // Apply a sensor model reading to the map
  void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance, double beamWidthInDegrees, double distanceErrorAsPercent) { 
    mapPanel.getMap().applySensorModelReading(sensorX, 
                                              sensorY,
                                              sensorAngle,
                                              distance,
                                              beamWidthInDegrees,
                                              distanceErrorAsPercent);
    recompute();                          
    mapPanel.update();
  }
  
  // Recompute the binary grid and borders if needed
  void recompute() { 
    try {
      if (Map.ShowBorders || Map.ShowBinary) 
        mapPanel.getMap().computeBinaryGrid();
      if (Map.ShowBorders) 
        mapPanel.getMap().computeBorders();
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }

  // Update the grid by applying the temp grid changes to the occupancy grid
  void updateGrid(double x, double y) { 
    mapPanel.getMap().updateGrid(x, y);
    recompute();
    mapPanel.update();
  }
  
  public static void main(String[] args) {
    MapperApp  mapper = new MapperApp(200, 150);
  }
}
