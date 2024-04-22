package uk.ac.ed.inf;

import uk.ac.ed.inf.ilp.constant.SystemConstants;
import uk.ac.ed.inf.ilp.data.*;

import java.util.*;

/**
 * a class to calculate the flight path
 */
public class FlightPathCalc {
    static LngLatHandler lngLatHandler = new LngLatHandler();
    //boolean value the check if the drone has entered central
    static boolean inCentral = false;

    /**
     * checks if a position is flying over no-fly zones
     * @param noFlyzones defined no-fly zones that the drone is banned to fly over
     * @param point the point to be checked
     * @return (@boolean true) if the point is in no-fly zone, (@boolean false) otherwise (valid)
     */
    public static boolean inNoFlyzones(NamedRegion[] noFlyzones, LngLat point) {
        for (NamedRegion zone : noFlyzones){
            if (lngLatHandler.isInRegion(point, zone)){
                return true;
            }
        }
        return false;
    }

    /**
     * get all possible direction from current position
     * @param position the current position.
     * @param noFlyZones defined no-fly zones
     * @param centralArea defined central area
     * @return (@List<LngLat> possible positions) child node of the current position is added to possible direction if it is in central and not in no-fly zone
     */
    private static List<LngLat> possibleDirection(LngLat position, NamedRegion[] noFlyZones, NamedRegion centralArea){
        List<LngLat> possiblePositions = new ArrayList<>();
        //loop through 16 possible directions
        for (double i=0; i<16; i++){
            //the child of the current position is calculated as the next position of the current position
            LngLat child = lngLatHandler.nextPosition(position, 22.5*i);
            //if the current position is in central but the chile is not in central, the child is removed from the possible positions list
            //because once the position enters central area, it cannot leave.
            if (inCentral && !pointInCentral(child, centralArea)){
                possiblePositions.remove(child);
            } else if (!inNoFlyzones(noFlyZones, child)){ //else if the child node is not in no-fly zone, add it to the possible positions list
                possiblePositions.add(child);
            }
        }
        return possiblePositions;
    }

    /**
     * checks if the point is in central
     * @param point the current position checked
     * @param centralArea the central area
     * @return (@boolean true) if the position is in central area (@boolean false) otherwise
     */
    private static boolean pointInCentral(LngLat point, NamedRegion centralArea){
        return lngLatHandler.isInCentralArea(point, centralArea);
    }


    /**
     * puts the calculated path into a list in order from start to end
     * @param visited - Map of visited points and their parents
     * @param goal - The goal position the algorithm is trying to get to. In this project, the goal is child of the visited position.
     * @return the reconstructed path
     */
    public static List<LngLat> reconstructPath(Map<LngLat,LngLat> visited, LngLat goal) {
        List<LngLat> path = new ArrayList<>();
        path.add(goal);
        while (visited.containsKey(goal)) {
            goal = visited.get(goal);
            //add the goal to the start of the path so that the path can be concatenated with reverse path later in the algorithm.
            path.add(0, goal);
        }
        return path;
    }

    /**
     * calculats the angle between two points
     * @param p1 one position checked
     * @param p2 the other position checked
     * @return (@double angle) is 999.0 if the two positions are equal. Otherwise, angle between two positions is calculated.
     */
    public static double angle(LngLat p1, LngLat p2){
        double angle;
        if (p1.equals(p2)){
            angle = 999.0;
        } else {
            double lng_diff = p2.lng() - p1.lng();
            double lat_diff = p2.lat() - p1.lat();
            angle = Math.toDegrees(Math.atan2(lat_diff, lng_diff));
        }

        if (angle < 0){
            //add 360 degree to the angle if it is a negative value so that the angle is in range of 0 degree through 360 degree.
            angle += 360.0;
        }

        return angle;
    }


    /**
     * the a* algorithm to find the flight path from start and end location.
     * @param noFlyzones defined no-fly zones that dronze cannot fly over
     * @param centralArea defined central area
     * @param start start position of the drone
     * @param end end positio of the drone
     * @return (@List<LngLat> reversePath) is the path from start to end and that from end to start so that the drone gets two-way path.
     */
    public static List<LngLat> FindPath(NamedRegion[] noFlyzones, NamedRegion centralArea, LngLat start, LngLat end) {

        // Initialize both open and close list
        Map<LngLat, Double> f = new HashMap<>(); // store f score of each lnglat positions: f = g+h
        //elements are ordered based on the values retrieved from f (ascending order - smaller f values have higher priority, come first in the queue)
        PriorityQueue<LngLat> openSet = new PriorityQueue<>(Comparator.comparingDouble(f::get));
        Map<LngLat, Double> g = new HashMap<>(); //store g score of each lnglat positions
        Set<LngLat> closedSet = new HashSet<>();
        Map<LngLat, LngLat> visited = new HashMap<>();
        List<LngLat> path = new ArrayList<>();
        List<LngLat> reversePath;

        //add the start node
        openSet.add(start);
        f.put(start, lngLatHandler.distanceTo(start, end));
        //add start gscore and set it to zero
        g.put(start, 0.0);

        //Loop until find the end
        while (!openSet.isEmpty()) {
            //Get the current node:
            //let the current position equal the node with the least f value
            //remove the currentNode from the openList
            LngLat currentPosition = openSet.poll(); //poll() define currentPosition as first element and remove from the list
            //add the currentPosition to the closedList
            if (lngLatHandler.isCloseTo(currentPosition, end)) {
                path = reconstructPath(visited, currentPosition);
                inCentral = false;
                break;
            }

            //once entered the central area the drone cannot leave
            if (!inCentral
                && currentPosition != start
                && lngLatHandler.isInCentralArea(currentPosition, centralArea)
                && !lngLatHandler.isInCentralArea(visited.get(currentPosition), centralArea)) {
                inCentral = true;
            }

            closedSet.add(currentPosition);
            //Generate children:
            //let the children of the currentPosition equal the adjacent positions
            List<LngLat> possiblePositions = possibleDirection(currentPosition, noFlyzones, centralArea);

            //for each child in the children
            for (LngLat position : possiblePositions) {
                // the position can be added to the path only when it is NOT in no-fly zones.
                    //child is on the closedList(visited): if position is in closedSet (visited) continue to beginning for loop
                if (closedSet.contains(position)) {
                    continue;
                }

                //create the f, g, and h values
                Double position_g = g.get(currentPosition) + SystemConstants.DRONE_MOVE_DISTANCE;
                //if position is in the openList's node positions
                if (openSet.contains(position)) {
                    //if the g score of the position is less than the one in g score map update the value with the calculated g value
                    if (position_g < g.get(position)) {
                        g.put(position, position_g); //update g score
                        visited.put(position, currentPosition); //update visited list, the child position as current position and current position as the parent position
                        Double existing_position_h = Math.abs(lngLatHandler.distanceTo(position, end)); //calculate h score
                        Double existing_position_f = position_g + existing_position_h; //calculate f score
                        f.put(position, existing_position_f); //update f score of the child position
                    }
                } else {
                    g.put(position, position_g);
                    visited.put(position, currentPosition);
                    Double position_h = Math.abs(lngLatHandler.distanceTo(position, end));
                    Double position_f = position_g + position_h;
                    f.put(position, position_f);
                    openSet.add(position); //add the position to openset with the updated f score

                }
            }

        }
        //reverse the path
        List<LngLat> pathCopy = new ArrayList<>(path); //copy the path calculated
        Collections.reverse(pathCopy); //reverse the copied path
        reversePath = pathCopy;
        reversePath.addAll(path); //concatenate reversed path and calculated path

        return reversePath;

    }
}