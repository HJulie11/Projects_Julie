package uk.ac.ed.inf;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import uk.ac.ed.inf.ilp.constant.OrderStatus;
import uk.ac.ed.inf.ilp.constant.OrderValidationCode;
import uk.ac.ed.inf.ilp.data.*;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Hello world!
 *
 */
public class App {

    //getting OrderValidator instance
    static OrderValidator orderValidator = new OrderValidator();

    /**
     * The function is checking if the order is valid
     * @param order
     * @return boolean value whether the order status is VALID_BUT_NOT_DELIVERED and order validation ncode is NO_ERROR
     */
    private static boolean isValidOrder(Order order){
        return order.getOrderStatus() == OrderStatus.VALID_BUT_NOT_DELIVERED
                && order.getOrderValidationCode() == OrderValidationCode.NO_ERROR;
    }

    public static void main( String[] args ) {

        LngLat appleton_tower = new LngLat(-3.186874, 55.944494);
        List<Deliveries> deliveries = new ArrayList<>();
        List<FlightPathResults> flightPathResultFile = new ArrayList<>();
        DroneGeoJSON droneGeoJSON = new DroneGeoJSON();
        GeoJSONFeature feature = new GeoJSONFeature();
        GeoJsonLineString lineString = new GeoJsonLineString();
        Map<Restaurant, List<LngLat>> pathToRestaurant = new HashMap<>();
        List<LngLat> path = new ArrayList<>();
        List<List<Double>> coordinates = new ArrayList<>();

        //check if the URL is provided
        if (args.length < 2){
            System.err.println("Base URL must be provided");
            System.exit(1);
        }
        try {
            var date = args[0];
            var baseUrl = args[1];
            //check if the provided URL is in the right format (the URL has to end with "/") - if not so, it adds "/"
            if (! baseUrl.endsWith("/")){
                baseUrl += "/";
            }

            /** checks if the provided date is in the right format with regex pattern
             * Year part: "^\\d{4}\\-" The date starts with 4 digits of year, followed by the hyphen '-'
             * Month part: "(0[1-9]|1[012])\-"
             * checks if the date matches the numbers 01 to 12 for months from January to December and follwed by the hyphen '-'
             * Day part: "(0[1-9]|[12][0-9]|3[01])"
             * checks if the date matches the numbers 01 to 31 of a month
             * '$': the date should end with the format above.
             */
            if (!date.matches("^\\d{4}\\-(0[1-9]|1[012])\\-(0[1-9]|[12][0-9]|3[01]$)")){
                System.err.println("Date Format is incorrect");
                System.exit(1);
            }

            //Initialize an instance of 'ObjectMapper' to use as mapper between JSON data and the data structures in this program.
            ObjectMapper mapper = new ObjectMapper();
            //Make the ObjectMapper understand serialization of LocalDate
            mapper.registerModule(new JavaTimeModule());
            mapper.enable(SerializationFeature.INDENT_OUTPUT);

            //Retrieve restaurants/orders/cenralArea/noFlyZones data separately from REST-server
            Restaurant[] restaurants = mapper.readValue(new URL(baseUrl + "restaurants"), new TypeReference<>() {
            });
            Order[] orders = mapper.readValue(new URL(baseUrl + "orders/" + date), new TypeReference<>() {
            });
            NamedRegion centralArea = mapper.readValue(new URL(baseUrl + "centralArea"), new TypeReference<>() {
            });
            NamedRegion[] noFlyZones = mapper.readValue(new URL(baseUrl + "noFlyZones"), new TypeReference<>() {
            });
            boolean isAlive = mapper.readValue(new URL(baseUrl + "isAlive"), new TypeReference<>() {
            });

            //check if the server is a alive - if not so, print an error and exit the program.
            if (!isAlive) {
                System.err.println("Server is not alive");
                System.exit(1);
            }

            //Calculate path from each restaurant to appleton and store it in a map
            for (Restaurant r : restaurants){
                pathToRestaurant.put(r, FlightPathCalc.FindPath(noFlyZones, centralArea, r.location(), appleton_tower));
            }

            //set the types and data
            lineString.setType("LineString");
            feature.setType("Feature");
            feature.setGeometry(lineString);
            feature.setProperties(new Properties());
            droneGeoJSON.setType("FeatureCollection");
            droneGeoJSON.addFeature(feature);

            //check if order is valid and add the data deliveries and flightpath
            for (Order order : orders){
                if (isValidOrder(orderValidator.validateOrder(order, restaurants))) {
                    Restaurant restaurantInOrder = orderValidator.getRestaurant(order, restaurants);
                    path = pathToRestaurant.get(restaurantInOrder);
                    if (path != null) {
                        order.setOrderStatus(OrderStatus.DELIVERED);
                    }
                    //each LngLat position in a path is added to the flightpath result file data format
                    //and added to the coordinates with the from longitude and latitude.
                    for (int i = 0; i < path.size()-1; i++){
                        flightPathResultFile.add(new FlightPathResults(order.getOrderNo(),
                                path.get(i).lng(),
                                path.get(i).lat(),
                                FlightPathCalc.angle(path.get(i), path.get(i+1)),
                                path.get(i+1).lng(),
                                path.get(i+1).lat()));
                        coordinates.add(List.of(path.get(i).lng(), path.get(i).lat()));

                    }
                    //this makes the last path of the order have same from and to lnglat, and make its angle 999
                    flightPathResultFile.add(new FlightPathResults(order.getOrderNo(),
                            path.get(path.size() - 1).lng(),
                            path.get(path.size() - 1).lat(),
                            999.0,
                            path.get(path.size() - 1).lng(),
                            path.get(path.size() - 1).lat()
                    ));
                    //sets the coordinates with the calculated coordinates
                    lineString.setCoordinates(coordinates);
                }
                //all orders are added to deliveries output file data
                deliveries.add(new Deliveries(order.getOrderNo(),
                        order.getOrderStatus(),
                        order.getOrderValidationCode(),
                        order.getPriceTotalInPence()));
            }

            String directory = "resultfiles"; //directory of the result files

            String geoJsonFileName = "drone-" + date + ".geojson";

            String deliveriesFileName = "deliveries-" + date + ".json";

            String flightPathFileName = "flightpath-" + date + ".json";

            mapper.writeValue(new File(directory, deliveriesFileName), deliveries);
            mapper.writeValue(new File(directory, flightPathFileName), flightPathResultFile);

            //when generating the file for a date with no order data, the file should only include an array
            if (orders.length == 0) {
                GsonBuilder gsonBuilder = new GsonBuilder().setPrettyPrinting();
                Gson gson = gsonBuilder.create();

                try (FileWriter write = new FileWriter("resultfiles/drone-" + date + ".geojson")){
                    //create an empty DroneGeoJSON object
                    DroneGeoJSON emptyData = new DroneGeoJSON();
                    // convert the empty DroneGeoJSON obejct into Json array
                    JsonArray droneGeoJson = gson.toJsonTree(emptyData).getAsJsonObject().getAsJsonArray("features");
                    gson.toJson(droneGeoJson, write);
                } catch (IOException e) {
                    e.printStackTrace();
                    throw e;
                }

            }else {
                //write the file
                mapper.writeValue(new File(directory, geoJsonFileName), droneGeoJSON);
            }

            System.out.println("JSON data has been written down successfully");
            System.out.println("GeoJSON data has been written down successfully");

        }
        catch (IOException e) {
            e.printStackTrace();
        }

    }
}
