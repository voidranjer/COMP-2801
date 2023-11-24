import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.awt.image.BufferedImage;

public class MapperPanel extends JPanel {
  private static final long serialVersionUID = 99999999;
  
  private static final Color   OBSTACLE_COLOR = new Color(150, 200, 255);       // light pale blue

  private Map          aMap;           // The model (i.e. the occupancyGrid with vector map as well)
  private AreaCoverage areaCoverage;   // The model for doing area coverage
  
  private int         width, height, magnification;
  private double      robotX, robotY;
  private double      resolution;
  
  private BufferedImage    backgroundCleaningImage;
  
  public MapperPanel(int w, int h, double res, int mag) {
    width = w; height = h; magnification = (int)(mag/res); resolution = res;
    //System.out.println("WINDOW HAS RESOLUTION ("+width*magnification+" x "+height*magnification+")");
    aMap = new Map(w, h, res);
    setBackground(Color.white);
  }

  public Map getMap() { return aMap; }

  public void setAreaCoverage(AreaCoverage ac) { areaCoverage = ac; }
  
  public void update() {
    repaint();
  }

  // This is the method that is responsible for displaying the map
  public void paintComponent(Graphics aPen) {
    super.paintComponent(aPen);
    VectorMap vmap = aMap.getVectorMap();
    vmap.draw(aPen, magnification);
    
    // Display the results of the area coverage algorithm which includes the obstacles, grid graph and spanning tree
  	try {
      // Display the obstacles
      if (MapperApp.ShowObstacles) {
        // Convert obstacles to polygons, then display the polygons
        int count = 0;
        
        for (Obstacle ob: vmap.getObstacles()) {
          Polygon p = new Polygon();
          Point avg = new Point();
          for (int i=0; i<ob.numVertices(); i++) {
            int x = MapperApp.MARGIN_X/2 + ob.getVertex(i).x*magnification;
            int y = MapperApp.MARGIN_Y/2 + vmap.getHeight() - ob.getVertex(i).y*magnification;
            p.addPoint(x, y);
            avg.x += x;
            avg.y += y;
          }
          aPen.setColor(OBSTACLE_COLOR);
          aPen.fillPolygon(p);
          aPen.setColor(Color.black);
          aPen.drawPolygon(p);
        }
      }
      // Display the robot's outer circle perimeter at each existing node location
      if (MapperApp.ShowPositions && (areaCoverage.getGridGraph() != null)) {
        for (Node n: areaCoverage.getGridGraph().getNodes()) {
          Point p = n.getLocation();
          aPen.setColor(Color.black);
          aPen.drawOval(MapperApp.MARGIN_X/2 + p.x*magnification - (int)(AreaCoverage.ROBOT_RADIUS*magnification),  
                        MapperApp.MARGIN_Y/2 + vmap.getHeight() - p.y*magnification - (int)(AreaCoverage.ROBOT_RADIUS*magnification), 
                        (int)(AreaCoverage.ROBOT_DIAMETER*magnification), (int)(AreaCoverage.ROBOT_DIAMETER*magnification));
        }
      }
      // Show the full graph or spanning tree if desired
      if ((MapperApp.ShowGridGraph || MapperApp.ShowSpanningTree) && (areaCoverage.getGridGraph() != null)) {
        areaCoverage.getGridGraph().draw(aPen, vmap.getHeight(), magnification);
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
