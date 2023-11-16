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
    
    allObstacles = new ArrayList<Obstacle>();
    convexObstacles = new ArrayList<Obstacle>();
    grownObstacles = new ArrayList<Obstacle>();
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
    grownObstacles.clear();
    reduced = false;
    triangulated = false;
  }
  
  public void loadTestWorld() {
    clear();
    Obstacle ob;
    ob = new Obstacle();
    ob.getVertices().add(new Point(10,225));
    ob.getVertices().add(new Point(5,216));
    ob.getVertices().add(new Point(52,190));
    ob.getVertices().add(new Point(22,138));
    ob.getVertices().add(new Point(38,128));
    ob.getVertices().add(new Point(68,180));
    ob.getVertices().add(new Point(109,157));
    ob.getVertices().add(new Point(113,165));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(70,120));
    ob.getVertices().add(new Point(70,110));
    ob.getVertices().add(new Point(110,110));
    ob.getVertices().add(new Point(110,60));
    ob.getVertices().add(new Point(32,60));
    ob.getVertices().add(new Point(32,50));
    ob.getVertices().add(new Point(132,50));
    ob.getVertices().add(new Point(132,60));
    ob.getVertices().add(new Point(120,60));
    ob.getVertices().add(new Point(120,120));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(157,96));
    ob.getVertices().add(new Point(135,76));
    ob.getVertices().add(new Point(150,60));
    ob.getVertices().add(new Point(122,33));
    ob.getVertices().add(new Point(150,5));
    ob.getVertices().add(new Point(198,53));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(170,170));
    ob.getVertices().add(new Point(170,117));
    ob.getVertices().add(new Point(130,117));
    ob.getVertices().add(new Point(130,107));
    ob.getVertices().add(new Point(170,107));
    ob.getVertices().add(new Point(170,100));
    ob.getVertices().add(new Point(180,100));
    ob.getVertices().add(new Point(180,170));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(71,267));
    ob.getVertices().add(new Point(61,250));
    ob.getVertices().add(new Point(181,180));
    ob.getVertices().add(new Point(201,214));
    ob.getVertices().add(new Point(132,253));
    ob.getVertices().add(new Point(122,236));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(234,294));
    ob.getVertices().add(new Point(220,280));
    ob.getVertices().add(new Point(241,258));
    ob.getVertices().add(new Point(221,238));
    ob.getVertices().add(new Point(252,207));
    ob.getVertices().add(new Point(217,172));
    ob.getVertices().add(new Point(223,165));
    ob.getVertices().add(new Point(258,200));
    ob.getVertices().add(new Point(270,188));
    ob.getVertices().add(new Point(297,216));
    ob.getVertices().add(new Point(270,245));
    ob.getVertices().add(new Point(284,259));
    ob.getVertices().add(new Point(263,280));
    ob.getVertices().add(new Point(255,273));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(240,167));
    ob.getVertices().add(new Point(240,150));
    ob.getVertices().add(new Point(200,150));
    ob.getVertices().add(new Point(200,110));
    ob.getVertices().add(new Point(240,110));
    ob.getVertices().add(new Point(240,77));
    ob.getVertices().add(new Point(279,77));
    ob.getVertices().add(new Point(279,27));
    ob.getVertices().add(new Point(289,27));
    ob.getVertices().add(new Point(289,87));
    ob.getVertices().add(new Point(250,87));
    ob.getVertices().add(new Point(250,167));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(338,261));
    ob.getVertices().add(new Point(338,182));
    ob.getVertices().add(new Point(328,182));
    ob.getVertices().add(new Point(328,163));
    ob.getVertices().add(new Point(288,163));
    ob.getVertices().add(new Point(288,153));
    ob.getVertices().add(new Point(314,153));
    ob.getVertices().add(new Point(314,131));
    ob.getVertices().add(new Point(275,131));
    ob.getVertices().add(new Point(275,111));
    ob.getVertices().add(new Point(314,111));
    ob.getVertices().add(new Point(314,4));
    ob.getVertices().add(new Point(324,4));
    ob.getVertices().add(new Point(324,153));
    ob.getVertices().add(new Point(348,153));
    ob.getVertices().add(new Point(348,180));
    ob.getVertices().add(new Point(378,180));
    ob.getVertices().add(new Point(378,190));
    ob.getVertices().add(new Point(348,190));
    ob.getVertices().add(new Point(348,240));
    ob.getVertices().add(new Point(378,240));
    ob.getVertices().add(new Point(378,250));
    ob.getVertices().add(new Point(348,250));
    ob.getVertices().add(new Point(348,261));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(0,299));
    ob.getVertices().add(new Point(0,0));
    ob.getVertices().add(new Point(2,0));
    ob.getVertices().add(new Point(2,299));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(8,299));
    ob.getVertices().add(new Point(8,297));
    ob.getVertices().add(new Point(198,297));
    ob.getVertices().add(new Point(198,299));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(8,2));
    ob.getVertices().add(new Point(8,0));
    ob.getVertices().add(new Point(197,0));
    ob.getVertices().add(new Point(197,2));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(202,299));
    ob.getVertices().add(new Point(202,297));
    ob.getVertices().add(new Point(392,297));
    ob.getVertices().add(new Point(392,299));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(205,2));
    ob.getVertices().add(new Point(205,0));
    ob.getVertices().add(new Point(395,0));
    ob.getVertices().add(new Point(395,2));
    allObstacles.add(ob);
    ob = new Obstacle();
    ob.getVertices().add(new Point(397,299));
    ob.getVertices().add(new Point(397,215));
    ob.getVertices().add(new Point(378,215));
    ob.getVertices().add(new Point(378,205));
    ob.getVertices().add(new Point(397,205));
    ob.getVertices().add(new Point(397,65));
    ob.getVertices().add(new Point(370,65));
    ob.getVertices().add(new Point(370,115));
    ob.getVertices().add(new Point(390,115));
    ob.getVertices().add(new Point(390,125));
    ob.getVertices().add(new Point(350,125));
    ob.getVertices().add(new Point(350,115));
    ob.getVertices().add(new Point(360,115));
    ob.getVertices().add(new Point(360,55));
    ob.getVertices().add(new Point(397,55));
    ob.getVertices().add(new Point(397,0));
    ob.getVertices().add(new Point(399,0));
    ob.getVertices().add(new Point(399,299));
    allObstacles.add(ob);
  }
}