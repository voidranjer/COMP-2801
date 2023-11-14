import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;

public class VectorMap {

  public static int         LINE_TOLERANCE;
  public static boolean     ShowVector = false;

  public static final int   VERTEX_RADIUS = 3;
  public static final int   VERTEX_DIAMETER = VERTEX_RADIUS*2;
  
  private static Color  VertexColor = Color.red;
  private static Color  EdgeColor = Color.blue;
  
  private int                   width;
  private int                   height;
  private double                resolution;
  private ArrayList<Obstacle>   allObstacles;        // The original obstacles
  private ArrayList<Obstacle>   convexObstacles;  // list of convex obstacles (with non-convex ones triangluated
  private boolean               reduced;       // true only if the map has been reduced by applying the line-fitting
  private boolean               triangulated;  // true only if the non-convex obstacles have been triangulated

  // Create a vector map with the given with, height,  resolution
  public VectorMap(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    reduced = true;
    triangulated = false;
    
    allObstacles = new ArrayList<Obstacle>();
    convexObstacles = new ArrayList<Obstacle>();
  }

  // Add an obstacle to the map
  public void addObstacle(Obstacle ob) { allObstacles.add(ob); }
  
  // Get methods
  public ArrayList<Obstacle> getObstacles() { 
    // Make sure that the polygon version of the obstacle has been computed
    for (Obstacle ob: allObstacles)
      ob.computePolygon();
    return allObstacles; 
  }
  
  // Get methods
  public ArrayList<Obstacle> getConvexObstacles() { 
    if (!triangulated) {
      triangulated = true;
      convexObstacles.clear();
      for (Obstacle ob: allObstacles) {
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
    allObstacles.clear();
    convexObstacles.clear();
    reduced = false;
    triangulated = false;
  }

  // Return the distance of point i to line ab
  private double distanceToLine(Point a, Point b, Point i) {
    return Math.abs((b.x - a.x)*(a.y-i.y) - (a.x-i.x)*(b.y-a.y)) / Math.sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  }
  
  // Apply the line tolerance to the obstacles so that there are a lot less vertices
  public void applyLineTolerance() {
    ArrayList<Obstacle>    newObstacles = new ArrayList<Obstacle>();
        
    for(Obstacle ob: allObstacles) {
      // Make sure there are at least 3 points, otherwise discard the data
      if (ob.numVertices() > 2) {
        Obstacle reducedOb = new Obstacle();
        // Get the first two points, add the first as the reduced obstacle's first vertex
        int a = 0;
        reducedOb.addVertex(ob.getVertex(0).x, ob.getVertex(0).y);
                
        // Now add the rest
        for (int b=1; b<ob.numVertices(); b++) {
          // Check if any points are too far away
          for (int i=a+1; i<b; i++) {
            if (distanceToLine(ob.getVertex(a), ob.getVertex(b), ob.getVertex(i)) > LINE_TOLERANCE) {
              // It is too far, then add the last point to the polygon and start a new line
              if (!((reducedOb.getVertex(reducedOb.numVertices()-1).x == ob.getVertex(b-1).x) && 
                    (reducedOb.getVertex(reducedOb.numVertices()-1).y == ob.getVertex(b-1).y)))
                reducedOb.addVertex(ob.getVertex(b-1).x, ob.getVertex(b-1).y);
              a = b-1;
            }
          }
        }
        // Replace the older obstacle with the reduced one, provided that it has at least three vertices
        if (reducedOb.numVertices() > 2)
          newObstacles.add(reducedOb);
       }
    }
    allObstacles = newObstacles;
    reduced = true;
  }
  
  // Open up a dialog box and get the name of a file and then save the vector map to that file.
  public void save() {
    JFileChooser chooser = new JFileChooser(new File("."));
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Vector Map File", "vmp");
    chooser.setFileFilter(filter);
    int returnVal = chooser.showSaveDialog(null);
    if (returnVal == JFileChooser.APPROVE_OPTION) {
      String fileName = chooser.getSelectedFile().getAbsolutePath();
      if (!fileName.toUpperCase().endsWith(".VMP"))
        fileName += ".vmp";
      try {
        DataOutputStream theFile  = new DataOutputStream(new FileOutputStream(fileName));
        theFile.writeInt(width);
        theFile.writeInt(height);
        theFile.writeDouble(resolution);
        theFile.writeInt(allObstacles.size());
        for (Obstacle ob:allObstacles) {
          theFile.writeInt(ob.getVertices().size());
          for (Point p: ob.getVertices()) {
            theFile.writeInt((int)(p.x*resolution));
            theFile.writeInt((int)(p.y*resolution));
          }
        }
        theFile.close();
      }
      catch(IOException e) {
        JOptionPane.showMessageDialog(null, "Error saving vector map file", "Error", JOptionPane.ERROR_MESSAGE);
      }
    }
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
            allObstacles.add(ob);
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
      for (Obstacle ob: allObstacles) {
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