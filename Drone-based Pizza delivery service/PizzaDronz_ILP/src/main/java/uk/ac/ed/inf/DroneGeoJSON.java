package uk.ac.ed.inf;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * a class for feature collection object.
 */
class DroneGeoJSON {
    private String type;
    private List<GeoJSONFeature> features;

    /**
     * Construct the feature collection objects with type "FeatureCollection" and features.
     */
    public DroneGeoJSON() {
        this.type = "FeatureCollection";
        this.features = new ArrayList<>();
    }

    public List<GeoJSONFeature> getFeatures() {
        return features;
    }

    public void setFeatures(List<GeoJSONFeature> features) {
        this.features = features;
    }

    public String getType(){
        return type;
    }
    public void setType(String type) {
        this.type = type;
    }

    /**
     * Adds feature to the features in feature collection.
     * @param feature feature is GeoJSONFeature object to be added to feature collection (DroneGeoJSON object)
     */
    public void addFeature(GeoJSONFeature feature) {
        this.features.add(feature);
    }
}

/**
 * a class of GeoJSON 'feature' type objects
 */
class GeoJSONFeature {
    private String type;
    private Geometry geometry;
    private Properties properties;

    public Geometry getGeometry() {
        return geometry;
    }

    public String getType(){
        return type;
    }
    public void setType(String type) {
        this.type = type;
    }

    public Properties getProperties() {
        return properties;
    }

    public void setGeometry(Geometry geometry) {
        this.geometry = geometry;
    }

    public void setProperties(Properties properties){
        this.properties = properties;
    }
}

/**
 * a class of Geometry objects
 */
class Geometry {
    private String type = "LineString"; // Geometry objects are in type "LineString"
    private List<List<Double>> coordinates; // List of coordinates
    public void setType(String type) {
        this.type = type;
    }

    public String getType(){
        return type;
    }
//
    public List<List<Double>> getCoordinates() {
        return coordinates;
    }

    /**
     * sets coordinates
     * @param coordinates coordinates are the Longitude and Latitude written in the format of '[Longitude, Latitude]'.
     */
    public void setCoordinates(List<List<Double>> coordinates) {
        this.coordinates = coordinates;
    }

}

/**
 * a class for GeoJSON Line String objects that extends Geometry class.
 */
class GeoJsonLineString extends Geometry {

}

/**
 * a class for Properties that extends HashMap of String and Object to print out 'properties' in output file.
 */
class Properties extends HashMap<String, Object> {

}

