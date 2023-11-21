import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import com.cyberbotics.webots.controller.Display;

public class VectorMap {

  private static final Color   OBSTACLE_COLOR = new Color(150, 200, 255);       // light pale blue

  private int                   width;
  private int                   height;
  private double                resolution;
  private ArrayList<Obstacle>   obstacles;        // The original obstacles
  private ArrayList<Obstacle>   convexObstacles;  // list of convex obstacles (with non-convex ones triangluated
  private boolean               triangulated;  // true only if the non-convex obstacles have been triangulated

  // Create a vector map with the given with, height,  resolution
  public VectorMap(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    triangulated = false;
    
    obstacles = new ArrayList<Obstacle>();
    convexObstacles = new ArrayList<Obstacle>();
  }

  // Add an obstacle to the map
  public void addObstacle(Obstacle ob) { obstacles.add(ob); }
   
  public ArrayList<Obstacle> getObstacles() { 
    // Make sure that the polygon version of the obstacle has been computed
    for (Obstacle ob: obstacles)
      ob.computePolygon();
    return obstacles; 
  }
  
  
  // Get methods
  public ArrayList<Obstacle> getConvexObstacles() { 
    if (!triangulated) {
      triangulated = true;
      convexObstacles.clear();
      for (Obstacle ob: obstacles) {
        if (ob.isConvex())
          convexObstacles.add(ob);
        else 
          convexObstacles.addAll(ob.splitIntoTriangles());
      }
      // Make sure that the polygon version of the obstacles have been computed
      for (Obstacle ob: convexObstacles)
        ob.computePolygon();
    }
    return convexObstacles; 
  }
  
  public int getWidth() { return width; }
  public int getHeight() { return height; }
  
  // Clear the obstacles from the map to get reqady for a new trace
  public void clear() {
    obstacles.clear();
    convexObstacles.clear();
    triangulated = false;
  }

  // Load up one of the 3 vector maps based on the name provided
  public void loadMapFromWorld(int mapID) {
    clear(); // Remove all the previous obstacles
    Obstacle ob = null;
    switch(mapID) {
      case 0:
        width = 200; height = 200; resolution = 0.5;
        ob = new Obstacle();
        ob.addVertex(0,10);
        ob.addVertex(0,0);
        ob.addVertex(199,0);
        ob.addVertex(199,10);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(0,199);
        ob.addVertex(0,189);
        ob.addVertex(199,189);
        ob.addVertex(199,199);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(0,199);
        ob.addVertex(0,0);
        ob.addVertex(10,0);
        ob.addVertex(10,199);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(189,199);
        ob.addVertex(189,0);
        ob.addVertex(199,0);
        ob.addVertex(199,199);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(10,95);
        ob.addVertex(10,55);
        ob.addVertex(30,55);
        ob.addVertex(30,95);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(10,55);
        ob.addVertex(10,35);
        ob.addVertex(110,35);
        ob.addVertex(110,55);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(90,110);
        ob.addVertex(90,90);
        ob.addVertex(190,90);
        ob.addVertex(190,110);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(70,189);
        ob.addVertex(70,139);
        ob.addVertex(120,139);
        ob.addVertex(120,189);
        obstacles.add(ob);
        break;
      case 1:
        width = 400; height = 300; resolution = 0.5;
        ob = new Obstacle();
        ob.addVertex(46,172);
        ob.addVertex(30,162);
        ob.addVertex(59,110);
        ob.addVertex(77,120);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(115,240);
        ob.addVertex(115,200);
        ob.addVertex(155,200);
        ob.addVertex(155,240);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(119,68);
        ob.addVertex(119,28);
        ob.addVertex(159,28);
        ob.addVertex(159,68);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(197,169);
        ob.addVertex(197,129);
        ob.addVertex(237,129);
        ob.addVertex(237,169);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(247,103);
        ob.addVertex(247,43);
        ob.addVertex(257,43);
        ob.addVertex(257,103);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(335,74);
        ob.addVertex(335,14);
        ob.addVertex(345,14);
        ob.addVertex(345,74);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(229,269);
        ob.addVertex(220,264);
        ob.addVertex(294,137);
        ob.addVertex(303,142);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(325,281);
        ob.addVertex(325,221);
        ob.addVertex(355,221);
        ob.addVertex(355,281);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(320,193);
        ob.addVertex(320,183);
        ob.addVertex(380,183);
        ob.addVertex(380,193);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(8,299);
        ob.addVertex(8,296);
        ob.addVertex(197,296);
        ob.addVertex(197,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(202,299);
        ob.addVertex(202,296);
        ob.addVertex(391,296);
        ob.addVertex(391,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(7,4);
        ob.addVertex(7,1);
        ob.addVertex(197,1);
        ob.addVertex(197,4);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(205,4);
        ob.addVertex(205,1);
        ob.addVertex(393,1);
        ob.addVertex(393,4);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(1,299);
        ob.addVertex(1,1);
        ob.addVertex(4,1);
        ob.addVertex(4,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(396,299);
        ob.addVertex(396,1);
        ob.addVertex(399,1);
        ob.addVertex(399,299);
        obstacles.add(ob);      
        break;
      case 2:
        width = 400; height = 300; resolution = 0.5;
        ob = new Obstacle();
        ob.addVertex(70,120);
        ob.addVertex(70,110);
        ob.addVertex(110,110);
        ob.addVertex(110,60);
        ob.addVertex(32,60);
        ob.addVertex(32,50);
        ob.addVertex(132,50);
        ob.addVertex(132,60);
        ob.addVertex(120,60);
        ob.addVertex(120,120);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(71,267);
        ob.addVertex(61,250);
        ob.addVertex(181,180);
        ob.addVertex(201,214);
        ob.addVertex(132,253);
        ob.addVertex(122,236);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(240,167);
        ob.addVertex(240,150);
        ob.addVertex(200,150);
        ob.addVertex(200,110);
        ob.addVertex(240,110);
        ob.addVertex(240,77);
        ob.addVertex(279,77);
        ob.addVertex(279,27);
        ob.addVertex(289,27);
        ob.addVertex(289,87);
        ob.addVertex(250,87);
        ob.addVertex(250,167);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(0,299);
        ob.addVertex(0,0);
        ob.addVertex(2,0);
        ob.addVertex(2,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(8,299);
        ob.addVertex(8,297);
        ob.addVertex(198,297);
        ob.addVertex(198,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(8,2);
        ob.addVertex(8,0);
        ob.addVertex(197,0);
        ob.addVertex(197,2);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(202,299);
        ob.addVertex(202,297);
        ob.addVertex(392,297);
        ob.addVertex(392,299);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(205,2);
        ob.addVertex(205,0);
        ob.addVertex(395,0);
        ob.addVertex(395,2);
        obstacles.add(ob);
        ob = new Obstacle();
        ob.addVertex(397,299);
        ob.addVertex(397,215);
        ob.addVertex(378,215);
        ob.addVertex(378,205);
        ob.addVertex(397,205);
        ob.addVertex(397,65);
        ob.addVertex(370,65);
        ob.addVertex(370,115);
        ob.addVertex(390,115);
        ob.addVertex(390,125);
        ob.addVertex(350,125);
        ob.addVertex(350,115);
        ob.addVertex(360,115);
        ob.addVertex(360,55);
        ob.addVertex(397,55);
        ob.addVertex(397,0);
        ob.addVertex(399,0);
        ob.addVertex(399,299);
        obstacles.add(ob);      
        break;
    } 
    System.out.println("Vector Map with " + obstacles.size() + " obstacles has been created");
  }
  
  
  
  // Open up a dialog box and get the name of a file and then load up the vector map from that file.
/*  public void load() {
    JFileChooser chooser = new JFileChooser(new File("."));
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Vector Map File", "vmp");
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
          System.out.println(newWidth+","+newHeight+","+newRes);
          if ((width != newWidth) || (height != newHeight) || (resolution != newRes)) {
            JOptionPane.showMessageDialog(null, "Error: Vector Map has different dimensions than current world","Error", JOptionPane.ERROR_MESSAGE);
            theFile.close();
            return;
          }
          clear();
          int count = theFile.readInt();
          for (int i=0; i<count; i++) {
            //System.out.println("        ob = new Obstacle();");
            Obstacle ob = new Obstacle();
            int points = theFile.readInt();

            for (int j=0; j<points; j++) { 
              Point p = new Point(theFile.readInt(),theFile.readInt());
              ob.getVertices().add(p);
              //System.out.println("        ob.addVertex("+p.x+","+p.y+");");
            }
            obstacles.add(ob);
            //System.out.println("        obstacles.add(ob);");
          }
          theFile.close();
        }
        catch(IOException e) {
          JOptionPane.showMessageDialog(null, "Error loading vector map file", "Error", JOptionPane.ERROR_MESSAGE);
        }
      }
      else
        JOptionPane.showMessageDialog(null, "Error: vector map file does not exist: " + fileName, "Error", JOptionPane.ERROR_MESSAGE);
     }
  }*/
  
  
  
  // Draw the vector map
  public void draw(Display aPen, int magnification) {
    /*try {
      // Convert obstacles to polygons, then display the polygons
      int count = 0;
      for (Obstacle ob: obstacles) {
        Polygon p = new Polygon();
        Point avg = new Point();
        for (int i=0; i<ob.size(); i++) {
          int x = (int)(ob.getVertex(i).x*magnification/resolution);
          int y = (int)(height - ob.getVertex(i).y*magnification/resolution);
          p.addPoint(x, y);
          avg.x += x;
          avg.y += y;
        }
        aPen.setColor(OBSTACLE_COLOR);
        aPen.fillPolygon(p);
        aPen.setColor(Color.black);
        aPen.drawPolygon(p);
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }*/
  }

}