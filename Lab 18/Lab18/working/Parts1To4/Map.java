import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;

public class Map {

  private static final float  ROBOT_RADIUS = 5.75f; // cm
  
  private VectorMap  vectorMap;     // a vector version of the map
  private int        width;
  private int        height;
  private double     resolution;
    
  // Create a map with the given with, height,  resolution
  public Map(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    vectorMap = new VectorMap(w, h, res);
  }

  public VectorMap getVectorMap() { return vectorMap; }
  

  public void draw(Graphics aPen, int magnification) {
    // Display the vector map version if desired
    if (VectorMap.ShowVector)
      vectorMap.draw(aPen, magnification);
  }
}
