import java.util.*;
import java.awt.*;
import java.io.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Trace {
  private int        width;
  private int        height;
  private double     resolution; 
  
  private ArrayList<Point>   actualPoints;
  private ArrayList<Point>   estimatePoints;
  private int[]              gridImage; // For display purposes only
  private int[]              estimateImage; // For display purposes only

  public Trace(int w, int h) { 
    actualPoints = new ArrayList<Point>();
    estimatePoints = new ArrayList<Point>();
    width = w;
    height = h;
    gridImage = new int[width*height];
    for (int y=0; y<height; y++) 
        for (int x=0; x<width; x++) 
          gridImage[y*width + x] = 0x00000000;
  }

  public void appendBoundaryPoint(int x, int y) {
    actualPoints.add(new Point(x, y));
    if (actualPoints.size() == 1) return;
    Point prev = actualPoints.get(actualPoints.size() - 2);
    Point current = actualPoints.get(actualPoints.size() - 1);
    for (int i=0; i<100; i++) {
      x = (int)((current.x - prev.x) * i/100.0 + prev.x);
      y = (int)((current.y - prev.y) * i/100.0 + prev.y);
      gridImage[(height/2-1-y)*width + x] = 0xFFFFFFFF;
     }
  }
  
  public void appendEstimatePoint(int x, int y) {
    estimatePoints.add(new Point(x, y));
    if (estimatePoints.size() == 1) return;
    Point prev = estimatePoints.get(estimatePoints.size() - 2);
    Point current = estimatePoints.get(estimatePoints.size() - 1);
    for (int i=0; i<100; i++) {
      x = (int)((current.x - prev.x) * i/100.0 + prev.x);
      y = (int)((current.y - prev.y) * i/100.0 + prev.y);
      if ((x>=0)&&(x<width)&&(y>=0)&&(y<height/2))
        gridImage[(height/2-1-y)*width + x] = 0xFFFF0000;
     }
  }

  public void draw(Display aPen) {
    ImageRef imageOfGrid1 = aPen.imageNew(width, height, gridImage, Display.ARGB);
    aPen.imagePaste(imageOfGrid1, 0, 0, false);
  }
}
