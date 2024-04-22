package uk.ac.ed.inf;
import org.junit.Test;
import uk.ac.ed.inf.ilp.data.*;
import uk.ac.ed.inf.ilp.constant.*;

import java.time.DayOfWeek;
import java.time.LocalDate;

import static org.junit.Assert.assertEquals;
public class TestOrderValidator {
    OrderValidator orderValidation = new OrderValidator();

    Restaurant[] restaurants = new Restaurant[]{
            new Restaurant(
                    "Civerinos Slice",
                    new LngLat(-3.1912869215011597, 55.945535152517735),
                    new DayOfWeek[]{DayOfWeek.MONDAY, DayOfWeek.TUESDAY, DayOfWeek.FRIDAY, DayOfWeek.SATURDAY, DayOfWeek.SUNDAY},
                    new Pizza[]{
                            new Pizza("Margarita", 1000),
                            new Pizza("Calzone", 1400)
                    }
            ),
            new Restaurant(
                    "Sora Lella Vegan Restaurant",
                    new LngLat(-3.202541470527649, 55.943284737579376),
                    new DayOfWeek[]{DayOfWeek.MONDAY, DayOfWeek.TUESDAY, DayOfWeek.WEDNESDAY, DayOfWeek.THURSDAY, DayOfWeek.FRIDAY},
                    new Pizza[]{
                            new Pizza("Meat Lover", 1400),
                            new Pizza("Vegan Delight", 1100)
                    }
            ),
            new Restaurant(
                    "Domino's Pizza - Edinburgh - Southside",
                    new LngLat(-3.1838572025299072, 55.94449876875712),
                    new DayOfWeek[]{DayOfWeek.WEDNESDAY, DayOfWeek.THURSDAY, DayOfWeek.FRIDAY, DayOfWeek.SATURDAY, DayOfWeek.SUNDAY},
                    new Pizza[]{
                            new Pizza("Super Cheese", 1400),
                            new Pizza("All Shrooms", 900)
                    }
            ),
            new Restaurant(
                    "Sodeberg Pavillion",
                    new LngLat(-3.1940174102783203, 55.94390696616939),
                    new DayOfWeek[]{DayOfWeek.TUESDAY, DayOfWeek.WEDNESDAY, DayOfWeek.SATURDAY, DayOfWeek.SUNDAY},
                    new Pizza[]{
                            new Pizza("Proper Pizza", 1400),
                            new Pizza("Pineapple & Ham & Cheese", 900)
                    }
            )
    };

    @Test
    public void testCardNumberInvalidLength() {
        Order order = new Order("ORD123", LocalDate.now(),OrderStatus.UNDEFINED,OrderValidationCode.UNDEFINED,1100,new Pizza[]{new Pizza("Margarita",1000)}, new CreditCardInformation("123456789012","12/25","123"));
        order.setCreditCardInformation(new CreditCardInformation("123456789012", "12/25", "123"));
        orderValidation.validateOrder(order,restaurants);

        assertEquals(OrderValidationCode.CARD_NUMBER_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testCardNumberIsNull() {
        Order order = new Order("ORD123", LocalDate.now(), OrderStatus.UNDEFINED, OrderValidationCode.UNDEFINED, 1100, new Pizza[]{new Pizza("Margarita", 1000)}, new CreditCardInformation(null, "12/25", "123"));
        order.setCreditCardInformation(new CreditCardInformation(null, "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.CARD_NUMBER_INVALID,order.getOrderValidationCode());
    }

    @Test
    public void testCardNumberNotDigits() {
        Order order = new Order("ORD123", LocalDate.now(), OrderStatus.UNDEFINED,OrderValidationCode.UNDEFINED, 1100, new Pizza[]{new Pizza("Margarita", 1000)},new CreditCardInformation("123456abcde12345", "12/25", "123"));
        order.setCreditCardInformation(new CreditCardInformation("123456abcde12345", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.CARD_NUMBER_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testCvvInvalidLength() {
        Order order = new Order("ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1100,
                new Pizza[]{new Pizza("Margarita", 1000)},
                new CreditCardInformation("4001919257537193",
                        "12/25",
                        "1234"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193",
                "12/25",
                "1234"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.CVV_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testCvvNotDigits() {
        Order order = new Order("ORD123", LocalDate.now(), OrderStatus.UNDEFINED, OrderValidationCode.UNDEFINED,1100, new Pizza[]{new Pizza("Margarita", 1000)}, new CreditCardInformation("4001919257537193", "12/25", "AB1"));
        order.setCreditCardInformation(new CreditCardInformation("4001919257537193","12/25","AB1"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.CVV_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testCvvNull(){
        Order order = new Order("ORD123", LocalDate.now(), OrderStatus.UNDEFINED, OrderValidationCode.UNDEFINED, 1100, new Pizza[]{new Pizza("Margarita", 1000)}, new CreditCardInformation("4001919257537193","12/25",null));
        order.setCreditCardInformation(new CreditCardInformation("4001919257537193","12/25",null));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.CVV_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testCardExpired() {
        Order order = new Order("ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1100,
                new Pizza[]{new Pizza("Meat Lover",
                        1000)},
                new CreditCardInformation("4001919257537193",
                        "12/25",
                        "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193",
                "12/15",
                "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.EXPIRY_DATE_INVALID, order.getOrderValidationCode());
    }
    @Test
    public void testCardExpiryNull() {
        Order order = new Order("ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1100,
                new Pizza[]{new Pizza("Margarita", 1000)},
                new CreditCardInformation("4001919257537193",
                        "12/25",
                        "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193",
                null,
                "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.EXPIRY_DATE_INVALID, order.getOrderValidationCode());
    }

    @Test
    public void testUndefinedPizza() {
        Order order = new Order("ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1100,
                new Pizza[]{new Pizza("Hello", 1000)},
                new CreditCardInformation("123456789012", "12/25", "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.PIZZA_NOT_DEFINED, order.getOrderValidationCode());
    }

    @Test
    public void testUndefinedSecondPizza() {
        Order order = new Order("ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                2500,
                new Pizza[]{new Pizza("Margarita", 1000),
                        new Pizza("Hello", 1400)},
                new CreditCardInformation("123456789012", "12/25", "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.PIZZA_NOT_DEFINED, order.getOrderValidationCode());
    }


    @Test
    public void testIncorrectTotal() {
        Order order = new Order("ORD123", LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1500,
                new Pizza[]{new Pizza("Margarita", 1000)},
                new CreditCardInformation("123456789012",
                        "12/25",
                        "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193",
                "12/25",
                "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.TOTAL_INCORRECT, order.getOrderValidationCode());
    }

    @Test
    public void testRestaurantClosed(){
        Order order = new Order("ORD123", LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                1100,
                new Pizza[]{new Pizza("Margarita", 1000)},
                new CreditCardInformation("123456789012",
                        "12/25",
                        "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193",
                "12/25",
                "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.RESTAURANT_CLOSED, order.getOrderValidationCode());
    }

    @Test
    public void testPizzaFromMultipleRestaurants() {
        Order order = new Order(
                "ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                2500,
                new Pizza[]{new Pizza("Margarita", 1000),
                        new Pizza("Meat Lover", 1400)},
                new CreditCardInformation("123456789012", "12/25", "123"));

        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.PIZZA_FROM_MULTIPLE_RESTAURANTS, order.getOrderValidationCode());
    }

    @Test
    public void testOrderLimitExceeded() {
        Order order = new Order(
                "ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                5100,
                new Pizza[]{new Pizza("Margarita", 1000),
                        new Pizza("Margarita", 1000),
                        new Pizza("Margarita", 1000),
                        new Pizza("Margarita", 1000),
                        new Pizza("Margarita", 1000)},
                new CreditCardInformation("123456789012", "12/25", "123"));
        order.setCreditCardInformation(new CreditCardInformation(
                "4001919257537193", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.MAX_PIZZA_COUNT_EXCEEDED, order.getOrderValidationCode());
    }

    @Test
    public void testNoError() {
        Order order = new Order(
                "ORD123",
                LocalDate.now(),
                OrderStatus.UNDEFINED,
                OrderValidationCode.UNDEFINED,
                2100,
                new Pizza[]{new Pizza("Meat Lover", 1000),
                        new Pizza("Meat Lover", 1000)},
                new CreditCardInformation("1234567890123456", "12/25", "123"));

        order.setCreditCardInformation(new CreditCardInformation(
                "1234567890123456", "12/25", "123"));
        orderValidation.validateOrder(order, restaurants);
        assertEquals(OrderValidationCode.NO_ERROR, order.getOrderValidationCode());
        assertEquals(OrderStatus.VALID_BUT_NOT_DELIVERED, order.getOrderStatus());
    }
}
