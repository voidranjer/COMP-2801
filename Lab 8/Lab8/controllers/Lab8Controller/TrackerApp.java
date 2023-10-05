import java.io.*;
import com.cyberbotics.webots.controller.Display;

public class TrackerApp {
  static int WindowWidth = 400;
  static int WindowHeight = 400;

  Display display;
  
  private Trace   trace;  // The model (i.e. the trace)

  public TrackerApp(Display d) {
    display = d;
    trace = new Trace(WindowWidth, WindowHeight);
  }
  
  void addActualLocation(int x, int y) { 
    trace.appendBoundaryPoint(x+WindowWidth/4, y+15+WindowHeight/4);
    trace.draw(display);
  }
  void addEstimatedLocation(int x, int y) { 
    trace.appendEstimatePoint(x+WindowWidth/4, y+15+WindowHeight/4);
    trace.draw(display);
  }
}
