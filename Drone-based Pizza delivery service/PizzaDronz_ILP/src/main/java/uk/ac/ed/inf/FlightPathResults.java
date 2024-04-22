package uk.ac.ed.inf;

/**
 * a class for flight path that will be printed out in the result file with its attributes.
 */
public class FlightPathResults {
    /* orderNo — the eight-character order number for the pizza order which the drone is currently collecting or delivering5;
    * a value (floating-point) fromLongitude — the longitude of the drone at the start of this move;
    * a value (floating-point) fromLatitude — the latitude of the drone at the start of this move;
    * a value (floating-point) angle — the angle of travel of the drone in this move6;
    * a value (floating-point) toLongitude — the longitude of the drone at the end of this move;
    * a value (floating-point) toLatitude — the latitude of the drone at the end of this move
     */
    String orderNo; //the order number of a pizza order that the drone is assigned for a delivery. This is a String length of 8.
    double fromLongitude; // the longitude of the start location of the drone
    double fromLatitude; // the latitude of the end location of the drone
    double angle; // the angle of the drone in the current move.
    double toLongitude; // the longitude of the end location of the drone
    double toLatitude; // the latitude of the end location of the drone

    /**
     * constructs the flight path objects to be put in a result file with attributed below:
     * @param orderNo (String of 8 characters) the order number of a pizza order
     * @param fromLongitude (double) the longitude of the start location of the drone
     * @param fromLatitude (double) the latitude of the start location of the drone
     * @param angle (double) the angle of the drone in the current move.
     * @param toLongitude (double) the longitude of the end location of the drone
     * @param toLatitude (double) the latitude of the end location of the drone
     */
    public FlightPathResults(String orderNo, double fromLongitude, double fromLatitude, double angle, double toLongitude, double toLatitude) {
        this.orderNo = orderNo;
        this.fromLongitude = fromLongitude;
        this.fromLatitude = fromLatitude;
        this.angle = angle;
        this.toLongitude = toLongitude;
        this.toLatitude = toLatitude;
    }

    // getters for each attribute to be used in a calculation and for the object mapper to read and write to a output file.
    public String getOrderNo() {
        return orderNo;
    }

    public double getFromLongitude() {
        return fromLongitude;
    }

    public double getFromLatitude() {
        return fromLatitude;
    }

    public double getAngle() {
        return angle;
    }

    public double getToLongitude() {
        return toLongitude;
    }

    public double getToLatitude() {
        return toLatitude;
    }

}
