import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class MapperPanel extends JPanel implements MouseMotionListener {
  private static final long serialVersionUID = 99999999;
  
  private Map          aMap;         // The model (i.e. the occupancyGrid with vector map as well)
  private PathPlanner  planner;      // The model for planning (i.e. the path planner)
  
  private int         width, height, magnification;
  private Point       mousePosition;
  private double      resolution;
  
  public MapperPanel(int w, int h, double res, int mag) {
    width = w; height = h; magnification = mag; resolution = res;
    System.out.println("WINDOW HAS RESOLUTION ("+width*magnification+" x "+height*magnification+")");
    aMap = new Map(w, h, res);
    setBackground(Color.white);
    addMouseMotionListener(this);
    mousePosition = new Point(0,0);
  }

  public Map getMap() { return aMap; }

  public void setPathPlanner(PathPlanner pp) { planner = pp; }
  
  public void mouseMoved(MouseEvent event) { 
    mousePosition = event.getPoint(); 
    update();
  }
  public void mouseDragged(MouseEvent event) { }
  public void mouseEntered(MouseEvent event) { }
  public void mouseExited(MouseEvent event) { }

  public void update() {
    repaint();
  }

  // This is the method that is responsible for displaying the map
  public void paintComponent(Graphics aPen) {
    super.paintComponent(aPen);
    aMap.draw(aPen, magnification);
    planner.displayResults(aPen, resolution, mousePosition);
  }
}
