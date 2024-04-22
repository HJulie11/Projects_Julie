//package uk.ac.ed.inf;
//
//import com.fasterxml.jackson.core.type.TypeReference;
//import com.fasterxml.jackson.databind.ObjectMapper;
//import org.junit.Test;
//import junit.framework.TestCase;
//import uk.ac.ed.inf.ilp.constant.OrderStatus;
//import uk.ac.ed.inf.ilp.constant.OrderValidationCode;
//import uk.ac.ed.inf.ilp.data.*;
//
//import java.io.IOException;
//import java.net.URL;
//import java.time.LocalDate;
//
//import static org.junit.Assert.assertEquals;
//
//public class AppTest extends TestCase {
//
//    @Test
//    public void TestRESTserver() throws IOException {
//        var baseUrl = "https://ilp-rest.azurewebsites.net";
//        ObjectMapper mapper = new ObjectMapper();
//        String date = "2023-11-15";
//        Restaurant[] restaurants = mapper.readValue(new URL(baseUrl + "restaurants"), new TypeReference<>() {
//        });
//        Order[] orders = mapper.readValue(new URL(baseUrl + "orders/" + date), new TypeReference<>() {
//        });
//        NamedRegion centralArea = mapper.readValue(new URL(baseUrl + "centralArea"), new TypeReference<>() {
//        });
//        NamedRegion[] noFlyZones = mapper.readValue(new URL(baseUrl + "noFlyZones"), new TypeReference<>() {
//        });
//        boolean isAlive = mapper.readValue(new URL(baseUrl + "isAlive"), new TypeReference<>() {
//        });
//
//        public void testTestRestaurantsNotNull() {
//            assertNotNull(restaurants);
//    }
//
//
//    }
//
//
//}
