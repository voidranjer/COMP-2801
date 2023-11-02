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
  
  static final int Magnification = 2;  // 4x magnification
  static int WindowWidth;
  static int WindowHeight;
  static int MARGIN_X = 10;
  static int MARGIN_Y = 10;
  
  private MapperPanel  mapPanel;

  public MapperApp(int widthInCentimeters, int heightInCentimeters) {
    super("Mapper App");

    WindowWidth = (int)(widthInCentimeters*Magnification*2)+ MARGIN_X*2;
    WindowHeight = (int)(heightInCentimeters*Magnification*2) + MARGIN_Y*2;
    
    add(mapPanel = new MapperPanel((int)(widthInCentimeters*Magnification*2), (int)(heightInCentimeters*Magnification*2), Magnification));


    // Create the menu bar and menus
    JMenuBar menuBar = new JMenuBar();
    setJMenuBar(menuBar);
    
    // Set up the Grid Map menu
    JMenu 	  mapMenu = new JMenu("GridMap"); menuBar.add(mapMenu);
    JMenuItem  loadGridMapButton = new JMenuItem("Load Grid Map");
    mapMenu.add(loadGridMapButton);
    loadGridMapButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map m = mapPanel.mapFromFile();
        WindowWidth = (int)(m.getWidth())+ MARGIN_X;
        WindowHeight = (int)(m.getHeight()) + MARGIN_Y;
        setSize(WindowWidth*Magnification+4, WindowHeight*Magnification+52);
        recompute();
        mapPanel.update();
      }});

    // Set up the Binary Threshold menu
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

    // Set up the Line Tolerance Menu
    JMenu	  toleranceMenu = new JMenu("LineTolerance"); menuBar.add(toleranceMenu);
    JCheckBoxMenuItem hideGrayButton = new JCheckBoxMenuItem("Hide Grayscale Map");  toleranceMenu.add(hideGrayButton);
    JCheckBoxMenuItem showVectorButton = new JCheckBoxMenuItem("Show Vector Map");  toleranceMenu.add(showVectorButton);
    showVectorButton.setSelected(VectorMap.ShowVector);
    SpinnerModel model2 = new SpinnerNumberModel(0, 0, 50, 1);
    JSpinner  toleranceSpinner = new JSpinner(model2);  toleranceMenu.add(toleranceSpinner);
    hideGrayButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowGrayscale = !hideGrayButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    showVectorButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        VectorMap.ShowVector = showVectorButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    toleranceSpinner.addChangeListener(new ChangeListener() {
      public void stateChanged(ChangeEvent e) {
        VectorMap.LINE_TOLERANCE = (int)(toleranceSpinner.getValue());
        recompute();
        mapPanel.update();
      }});

    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setSize(WindowWidth+4, WindowHeight+52);
    setVisible(true);
  }
  
  // Recompute the binary grid and vector map if needed
  void recompute() { 
    try {
      if (Map.ShowBorders || Map.ShowBinary || VectorMap.ShowVector) 
        mapPanel.getMap().computeBinaryGrid();
      if (Map.ShowBorders || VectorMap.ShowVector) 
        mapPanel.getMap().computeBorders();
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
  
  public static void main(String[] args) {
    MapperApp  mapper = new MapperApp(200, 150);
  }
}
