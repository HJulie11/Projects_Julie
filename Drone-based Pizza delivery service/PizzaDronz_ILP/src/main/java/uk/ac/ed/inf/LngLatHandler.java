package uk.ac.ed.inf;

import uk.ac.ed.inf.ilp.constant.SystemConstants;
import uk.ac.ed.inf.ilp.data.LngLat;
import uk.ac.ed.inf.ilp.data.NamedRegion;
import uk.ac.ed.inf.ilp.interfaces.LngLatHandling;

/**
 * class of point in a vertex
 * this will be used to compare the vertices of region and the position
 */
class Point {
    double x, y;
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
}

/**
 * a class implementing functions from LngLatHandling interface:
 * to calculate distance between two positions
 * to check if two positions are close to each other
 * to check if a position is in a region
 * and to calculate the next position with the allowed angle and distance for one move of a drone
 */
public class LngLatHandler implements LngLatHandling {

    /**
     * finds the distance between two positions.
     * @param startPosition
     * @param endPosition
     * @return value of calculated distance between two positions
     */
    public double distanceTo(LngLat startPosition, LngLat endPosition) {
        //difference between the longitudes of the two given positions: (x2-x1)
        double lngDistance = endPosition.lng() - startPosition.lng();
        //difference between the latitudes of the two given positions: (y2-y1)
        double latDistance = endPosition.lat() - startPosition.lat();

        //sqrt((x2-x1)^2 + (y2-y1)^2)
        return Math.sqrt((lngDistance*lngDistance)+(latDistance*latDistance));
    }

    /**
     * check if the two positions are close to each other using system constant DRONE_IS_CLOSE_DISTANCE
     * @param startPosition
     * @param otherPosition
     * @return (@boolean true) if the distance between two positions are less than the defined close distance between two position.
     */
    public boolean isCloseTo(LngLat startPosition, LngLat otherPosition) {
        double distanceBetween = distanceTo(startPosition, otherPosition);

        // if the distance between the two position is smaller than the constant, they are considered as close positions.
        return distanceBetween <= SystemConstants.DRONE_IS_CLOSE_DISTANCE;
    }

    /**
     * check if the given position is in given region using the vertices of region.
     * @param position position to be checked
     * @param region defined region
     * @return (@pointInside true) if position matches any of the vertices of the region.
     */
    public boolean isInRegion(LngLat position, NamedRegion region) {
        LngLat[] regionVertices = region.vertices();

        //number of vertices
        int verticesLength = regionVertices.length;

        //Values of Position in double type
        double lng = position.lng();
        double lat = position.lat();
        
        // as the regions have vertices, 
        Point[] pointVertices = new Point[regionVertices.length];

        //Convert the list of LngLat types to list of points
        for (int i=0; i < verticesLength; i++){ //i=1?
            Point p = new Point(regionVertices[i].lng(), regionVertices[i].lat());
            pointVertices[i] = p;
        }

        boolean pointInside = false;

        //First vertex
        Point firstVertex = pointVertices[0];

        //Will loop through each vertex in the polygon
        for (int i = 1; i < pointVertices.length; i++){

            //Current vertices
            Point vertex = pointVertices[i%pointVertices.length];

            //checks if the point is lower than the bottom of the polygon
            if (lat > Math.min(firstVertex.y, vertex.y)) {
                //checks if the point is higher the top of the polygon
                if (lat <= Math.max(firstVertex.y, vertex.y)){
                    //checks if the point is on the left side of the polygon
                    if (lng <= Math.max(firstVertex.x, vertex.x)){
                        //calculates where line passing through the two points and the horizontal line passing through the vertices intersect with each other
                        double intersectionOfX = (lat - firstVertex.y) * (vertex.x - firstVertex.x)/(vertex.y - firstVertex.y) + firstVertex.x;

                        //checks if the two points have the same x coordinate or if the line is on the vertices
                        if (firstVertex.x == vertex.x || lng < intersectionOfX){
                            pointInside = !pointInside;
                        }
                    }
                }
            }

            firstVertex = vertex;
        }

        return pointInside;
    }

    /**
     * find the next position considering that one move takes 22.5 degree
     * @param startPosition
     * @param angle
     * @return the next position in longitude and latitude format
     */
    public LngLat nextPosition(LngLat startPosition, double angle) {
        if ((angle % 22.5 != 0) || (angle > 360)){
            return null;
        } else{
            //Calculated the distance it travelled horizontally and vertically
            double lat = SystemConstants.DRONE_MOVE_DISTANCE * Math.sin(Math.toRadians(angle));
            double lng = SystemConstants.DRONE_MOVE_DISTANCE * Math.cos(Math.toRadians(angle));

            //Add the distance travelled to the start position coordinate to get the new coordinates
            return new LngLat(startPosition.lng() + lng, startPosition.lat() + lat);
        }
    }
}
