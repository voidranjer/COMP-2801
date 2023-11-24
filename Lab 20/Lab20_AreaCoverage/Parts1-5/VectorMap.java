import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;

public class VectorMap {

  public static boolean     ShowVector = false;

  public static final int   VERTEX_RADIUS = 3;
  public static final int   VERTEX_DIAMETER = VERTEX_RADIUS*2;
  
  private static Color  VertexColor = Color.red;
  private static Color  EdgeColor = Color.blue;
  
  private int                   width;
  private int                   height;
  private double                resolution;
  private ArrayList<Obstacle>   obstacles;        // The original obstacles
  private ArrayList<Obstacle>   convexObstacles;  // list of convex obstacles (with non-convex ones triangluated
  private ArrayList<Obstacle>   grownObstacles;   // List of obstacles after they have been grown
  private boolean               reduced;       // true only if the map has been reduced by applying the line-fitting
  private boolean               triangulated;  // true only if the non-convex obstacles have been triangulated

  // Create a vector map with the given with, height,  resolution
  public VectorMap(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    reduced = true;
    triangulated = false;
    
    obstacles = new ArrayList<Obstacle>();
    convexObstacles = new ArrayList<Obstacle>();
    grownObstacles = new ArrayList<Obstacle>();
  }

  // Add an obstacle to the map
  public void addObstacle(Obstacle ob) { obstacles.add(ob); }
  
  // Get methods
  public double getResolution() { return resolution; }
  
  public ArrayList<Obstacle> getObstacles() { 
    // Make sure that the polygon version of the obstacle has been computed
    for (Obstacle ob: obstacles)
      ob.computePolygon();
    return obstacles; 
  }
  
  public ArrayList<Obstacle> getGrownObstacles() { 
    // Make sure that the polygon version of the obstacle has been computed
    for (Obstacle ob: grownObstacles)
      ob.computePolygon();
    return grownObstacles; 
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
    grownObstacles.clear();
    reduced = false;
    triangulated = false;
  }

  // Return the distance of point i to line ab
  private double distanceToLine(Point a, Point b, Point i) {
    return Math.abs((b.x - a.x)*(a.y-i.y) - (a.x-i.x)*(b.y-a.y)) / Math.sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  }

  // Load up one of the 3 vector maps based on the name provided
  public void loadMapFromWorld() {
    clear(); // Remove all the previous obstacles
    Obstacle ob = new Obstacle();
    ob.addVertex(27,285);
    ob.addVertex(27,265);
    ob.addVertex(48,265);
    ob.addVertex(48,285);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(111,102);
    ob.addVertex(40,32);
    ob.addVertex(48,24);
    ob.addVertex(118,95);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(115,240);
    ob.addVertex(115,200);
    ob.addVertex(155,200);
    ob.addVertex(155,240);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(97,134);
    ob.addVertex(97,129);
    ob.addVertex(157,129);
    ob.addVertex(157,134);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(197,169);
    ob.addVertex(197,129);
    ob.addVertex(237,129);
    ob.addVertex(237,169);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(193,112);
    ob.addVertex(184,107);
    ob.addVertex(204,72);
    ob.addVertex(213,77);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(291,70);
    ob.addVertex(286,52);
    ob.addVertex(305,46);
    ob.addVertex(310,65);
    obstacles.add(ob);
    ob = new Obstacle();
    ob.addVertex(346,162);
    ob.addVertex(337,157);
    ob.addVertex(357,122);
    ob.addVertex(365,127);
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
    
    System.out.println("Vector Map with " + obstacles.size() + " obstacles has been created");
  }
  

  // Open up a dialog box and get the name of a file and then load up the vector map from that file.
  public void load() {
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
          if ((width != newWidth) || (height != newHeight) || (resolution != newRes)) {
            JOptionPane.showMessageDialog(null, "Error: Vector Map has different dimensions than current world","Error", JOptionPane.ERROR_MESSAGE);
            theFile.close();
            return;
          }
          clear();
          int count = theFile.readInt();
          for (int i=0; i<count; i++) {
            Obstacle ob = new Obstacle();
            int points = theFile.readInt();

            for (int j=0; j<points; j++) { 
              Point p = new Point(theFile.readInt(),theFile.readInt());
              ob.getVertices().add(p);
            }
            obstacles.add(ob);
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
  }
  
  
  
  // Draw the vector map
  public void draw(Graphics aPen, int magnification) {
    try {
      // Display the obstacles
      if (!reduced) return;  /// Do not display the map if not yet reduced
      for (Obstacle ob: obstacles) {
        ArrayList<Point>  vertices = ob.getVertices();
        
        // Draw the edges first
        for (int i=0; i<vertices.size()-1; i++) {
            aPen.setColor(EdgeColor);
            aPen.drawLine(MapperApp.MARGIN_X/2 + vertices.get(i).x, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y, 
                          MapperApp.MARGIN_X/2 + vertices.get(i+1).x, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i+1).y);
        }
        aPen.drawLine(MapperApp.MARGIN_X/2 + vertices.get(0).x, 
                      MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(0).y, 
                      MapperApp.MARGIN_X/2 + vertices.get(vertices.size()-1).x, 
                      MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(vertices.size()-1).y);
        
        // Now Draw the vertices
        for (int i=0; i<vertices.size(); i++) {
            aPen.setColor(VertexColor);
            aPen.fillOval(MapperApp.MARGIN_X/2 + vertices.get(i).x-VERTEX_RADIUS, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y-VERTEX_RADIUS, 
                          VERTEX_DIAMETER, VERTEX_DIAMETER);
            aPen.setColor(Color.black);
            aPen.drawOval(MapperApp.MARGIN_X/2 + vertices.get(i).x-VERTEX_RADIUS, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y-VERTEX_RADIUS, 
                          VERTEX_DIAMETER, VERTEX_DIAMETER);
        }
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }

}