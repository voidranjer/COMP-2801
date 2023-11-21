import java.io.*;
import java.awt.Dimension;
import com.cyberbotics.webots.controller.Display;

public class MapperApp {
  
  static final float Resolution = 0.5f; // 0.5 cm per grid cell
  static final int Magnification = 2;  // 2x magnification
  static int WindowWidth;
  static int WindowHeight;
  
  private Map                 aMap;              // The model (i.e. the occupancyGrid)
  private Display             display;           // inner display window of Webots that will display the map
  private LocationEstimator   locationEstimator; // object that does location estimation

  public MapperApp(int widthInCentimeters, int heighInCentimeters, int mapID, Display d) {
    display = d;
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification);
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification);

    aMap = new Map((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution);
    aMap.getVectorMap().loadMapFromWorld(mapID);
    locationEstimator = new LocationEstimator(aMap.getVectorMap());
  }
  
  // Update the distance travelled in the location Estimates
  void updateDistanceInLocationEstimator(double amount) {
    try {
      locationEstimator.updateLocation(amount);
    }
    catch (java.util.ConcurrentModificationException e) {}
    locationEstimator.draw(display, (int)(Magnification/Resolution));
  }
  
  // Update the amount turned in the location Estimates
  void updateOrientationInLocationEstimator(double amount) {
    try {
      locationEstimator.updateOrientation(amount);
    }
    catch (java.util.ConcurrentModificationException e) {}
    locationEstimator.draw(display, (int)(Magnification/Resolution));
  }
  
  // Update the location estimates based on the distance reading
  void updateReadingInLocationEstimator(double distanceReading) {
    try {
      locationEstimator.estimateFromReading(distanceReading);
    }
    catch (java.util.ConcurrentModificationException e) {}
    locationEstimator.draw(display, (int)(Magnification/Resolution));
  }
}