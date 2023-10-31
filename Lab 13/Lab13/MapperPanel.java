import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.*;

public class MapperPanel extends JPanel {
  private static final long serialVersionUID = 99999999;
  
  private Map   aMap;  // The model (i.e. the occupancyGrid)
  private int   width, height, magnification;
  
  public MapperPanel(int w, int h, double res, int mag) {
    width = w; height = h; magnification = mag;
    System.out.println("WINDOW HAS RESOLUTION ("+width*magnification+" x "+height*magnification+")");
    aMap = new Map(w, h, res);
    setBackground(Color.white);
  }

  public Map getMap() { return aMap; }
  public void setMap(Map m) { aMap = m; update(); }

  public void update() {
    repaint();
  }

  // This is the method that is responsible for displaying the map
  public void paintComponent(Graphics aPen) {
    super.paintComponent(aPen);
    aMap.draw(aPen, magnification);
  }
  
  public void createNewMap(int newHeight, int newWidth, double res) {
    width = newWidth; height = newHeight;
    aMap = new Map(width, height, res);
  }
  
    // Open up a dialog box and get the name of a file and then load up the occupancy grid from that file.
  public Map mapFromFile() {
    JFileChooser chooser = new JFileChooser(new File("."));
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Map File", "map");
    chooser.setFileFilter(filter);
    int returnVal = chooser.showOpenDialog(null);
    if (returnVal == JFileChooser.APPROVE_OPTION) {
      String fileName = chooser.getSelectedFile().getAbsolutePath();
      if (new File(fileName).exists()) {
      	try {
      	  DataInputStream theFile  = new DataInputStream(new FileInputStream(fileName));
      	  int newWidth = theFile.readInt();
      	  int newHeight = theFile.readInt();
      	  double newRes = theFile.readDouble();
      	  System.out.println(newWidth + "," + newHeight + "," + newRes);
      	  aMap = new Map(newWidth, newHeight, newRes);
      	  aMap.readRestFromFile(theFile);
      	  theFile.close();
      	  return aMap;
      	} 
    	catch(IOException e) {
      	  JOptionPane.showMessageDialog(null, "Error loading map file", "Error", JOptionPane.ERROR_MESSAGE);
      	  return aMap;
    	}
      }
      else {
        JOptionPane.showMessageDialog(null, "Error: map file does not exist: " + fileName, "Error", JOptionPane.ERROR_MESSAGE);
		return aMap;
	  }
    }
    return aMap;
  }
}
