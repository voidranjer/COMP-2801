import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class Map {
  private static final float  LIGHTEN_FACTOR = 20;
  private static final float  ROBOT_RADIUS = 5.75f; // cm
  
  static boolean  ShowGrayscale = true;
  static boolean  ShowBinary = false;
  static boolean  ShowBorders = false;
  static int      BINARY_THRESHOLD;
  
  private static Color[] greyScalePalette = new Color[256];
  private VectorMap  vectorMap;     // a vector version of the map
  private int        width;
  private int        height;
  private double     resolution;
  private float      maxValue = 0.0f;
  private float      minValue = 0.5f;
    
  // Create a map with the given with, height,  resolution
  public Map(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    vectorMap = new VectorMap(w, h, res);
    
    // Create Grayscale Palette
    for (int i=0; i<=255; i++) 
      greyScalePalette[i] = new Color(255-i, 255-i, 255-i);
  }

  public VectorMap getVectorMap() { return vectorMap; }
  

  
  public void draw(Graphics aPen, int magnification) {
    try {
      // Display the vector map version if desired
      if (VectorMap.ShowVector) {
        vectorMap.draw(aPen, magnification);
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
