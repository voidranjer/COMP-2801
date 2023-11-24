import java.awt.*;
import java.io.*;
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class MapperApp{
  private static final Color   OBSTACLE_COLOR = new Color(150, 200, 255);       // light pale blue
  
  static final float    Resolution = 0.5f; // 0.5 cm per grid cell
  static final int      Magnification = 1;   // 2x magnification
  static int            WindowWidth;
  static int            WindowHeight;
  
  private Display       display;             // inner display window of Webots that will display the map
  private int           DISPLAY_COUNTER = 0; // This is used to limit the number of times to display

  // This is the area coverage that computes the are coverage grid graph and spanning tree from the vector map in the mapPanel
  private AreaCoverage  areaCoverage;
  public  AreaCoverage  getAreaCoverage() { return areaCoverage; }
  
  private Map           map;
  private int[]         gridImage;        // For display purposes only

  public static boolean     ShowObstacles = false;
  public static boolean     ShowPositions = false;
  public static boolean     ShowGridGraph = false;
  public static boolean     ShowSpanningTree = true;
  
  
  
  // Constructor to create the MapperApp with the webots display
  public MapperApp(int widthInCentimeters, int heighInCentimeters, Display d) {
    display = d;
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification);
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification);
    
    map = new Map((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution);

    // Start with an area coverage based on an empty vector map to start
    VectorMap vmap = map.getVectorMap();
    areaCoverage = new AreaCoverage(vmap); 
    
    // This is for display purposes only
    gridImage = new int[vmap.getWidth()*vmap.getHeight()];
    for (int y=0; y<vmap.getHeight(); y++) {
      for (int x=0; x<vmap.getWidth(); x++) 
         gridImage[y*vmap.getWidth() + x] = 0xAAAAAAAA;
    }
  }
  
  // Draw the spanning tree on the display
  public void drawSpanningTree() {
    // Draw the spanning tree
    ArrayList<Edge>    edges = areaCoverage.getGridGraph().getEdges();

    int width = map.getVectorMap().getWidth();
    int height = map.getVectorMap().getHeight();
    for (Edge e: edges) {  // Draw the edges first
      if (e.isSelected()) {
        float d = e.length();
        int sx = e.getStartNode().getLocation().x;
        int sy = e.getStartNode().getLocation().y;
        int ex = e.getEndNode().getLocation().x;
        int ey = e.getEndNode().getLocation().y;
        int xDiff = ex - sx;
        int yDiff = ey - sy;
        for (int i=0; i<d; i++) {
          int px = sx + (int)(((float)i/d)*xDiff);
          int py = sy + (int)(((float)i/d)*yDiff);
          gridImage[(int)(height*Resolution - py)*width + px] = 0xFFFFFFFF;
        }
      }
    }
  }
  
  
  // Get the vector map
  public VectorMap getVectorMap() {
    return map.getVectorMap();
  }
  
  public void showFinalTree() {
    VectorMap vmap = map.getVectorMap();
    drawSpanningTree();
    ImageRef imageOfGrid = display.imageNew(vmap.getWidth(), vmap.getHeight(), gridImage, Display.ARGB);
    display.imagePaste(imageOfGrid, 0, 0, false);
  }
 
  public void updatePosition(double x, double y) {
    VectorMap vmap = map.getVectorMap();
    // Display the position of the robot
    try {
      // Clear around the robot
      float inc = (float)Math.PI/180.0f;
      for (float a=0; a<Math.PI*2; a+=inc) {
        for (float d=0; d<AreaCoverage.ROBOT_RADIUS; d+=0.5f) {
          float px = (float)(x+d*Math.cos(a));
          float py = (float)(y+d*Math.sin(a));
          gridImage[(int)(vmap.getHeight()*Resolution - py)*vmap.getWidth() +  (int)px] = 0;
        }
      }       
      if (DISPLAY_COUNTER++ %25 != 0)
        return;
      ImageRef imageOfGrid = display.imageNew(vmap.getWidth(), vmap.getHeight(), gridImage, Display.ARGB);
      display.imagePaste(imageOfGrid, 0, 0, false);
      
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
    drawSpanningTree();
  }
}
