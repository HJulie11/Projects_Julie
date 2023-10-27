package external;

import com.graphhopper.ResponsePath;
import com.graphhopper.util.Translation;
import com.graphhopper.util.shapes.GHPoint;
import model.TransportMode;

/**
 * Interface for the {@link MapSystem}. It allows requesting for directions between two address
 * given as coordinates. There is only one {@link MapSystem} and all users of this application use
 * the same system.
 */
public interface MapSystem extends AutoCloseable {
  static String validCoordString = "^\\d+(\\.\\d+)?\\s-?\\d+(\\.\\d+)?$";

  /**
   * Convert a string containing some coordinates into a {@link GHPoint} object
   *
   * @param address string containing the latitude and longitude
   * @return The {@link GHPoint} object containing the given latitude and longitude
   */
  GHPoint convertToCoordinates(String address);

  /**
   * Check if a point is within the bounds of the map
   *
   * @param point point to check
   * @return True if point is within map, false otherwise
   */
  boolean isPointWithinMapBounds(GHPoint point);

  /**
   * Generate the route between two points on the map
   *
   * @param mode transportation mode to generate route for
   * @param startPoint the starting point
   * @param destination the ending point
   * @return The route to get from startPoint to endPoint.
   */
  ResponsePath routeBetweenPoints(TransportMode mode, GHPoint startPoint, GHPoint destination);

  /**
   * @return The map translate into a UK locale
   */
  Translation getTranslation();
}
