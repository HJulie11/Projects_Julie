package uk.ac.ed.inf;

import uk.ac.ed.inf.ilp.interfaces.OrderValidation;
import uk.ac.ed.inf.ilp.data.*;
import uk.ac.ed.inf.ilp.constant.*;

import java.time.LocalDate;
import java.util.*;



public class OrderValidator implements OrderValidation {
    HashMap<String, Restaurant> menus = new HashMap<>();

    /**
     *
     * @param creditCardInformation the credit card information consists of expiry date, credit card number, cvv digits.
     * @param orderDate the expiry date is compared with the order date
     * @return boolean value if the card is expired or not
     */
    //helper function return if the expiry date is after current date
    private boolean expiryDateValid(CreditCardInformation creditCardInformation, LocalDate orderDate) {
        String expiryDate = creditCardInformation.getCreditCardExpiry();

        //check if the expiry date is null or not in the right format
        if (expiryDate == null || !(expiryDate.matches("(?:0[1-9]|1[0-2])/[0-9]{2}"))){
            return false;
        }
        String[] parts = expiryDate.split("/");
        int expMonth = Integer.parseInt(parts[0]);
        int expYear = Integer.parseInt(parts[1])+2000;

        //Get the month and year from the order date
        int currentMonth = orderDate.getMonthValue();
        int currentYear = orderDate.getYear();

        //Check if the given expiry date is after the order date
        if (expYear < currentYear){
            return false;
        } else if (expYear == currentYear) {
            return expMonth > currentMonth;
        } else {
            return true;
        }

    }

    /**
     *
     * @param order order to be validated with the total cost
     * @return boolean value whether the total cost is correctly or not.
     */
    private boolean totalCost(Order order) {
        int priceTotalInPence = order.getPriceTotalInPence();
        int totalCalculated = 100;
        Pizza[] pizzasInOrder = order.getPizzasInOrder();
        boolean total_valid = true;

        for (Pizza p : pizzasInOrder) {
            int pizzaPrice = p.priceInPence();
            totalCalculated += pizzaPrice;
        }
        if (totalCalculated != priceTotalInPence) {
            total_valid = false;
        }

        return total_valid;
    }

    /**
     *
     * @param order order to be validated on the information about the pizza in the placed order
     * @param restaurants defined restaurants on the REST server
     * @return (@pizzaDefined true) if the pizza is in menus of on of the restaurans.
     */
    private boolean pizzaExists(Order order, Restaurant[] restaurants) {
        Pizza[] orderedPizzas = order.getPizzasInOrder();
        boolean pizzaDefined = false;
        for (Restaurant r : restaurants) {
            for (Pizza pizzaMenu : r.menu()) {
                menus.put(pizzaMenu.name(), r);
            }
        }

        for (Pizza pizza : orderedPizzas) {
            pizzaDefined = menus.containsKey(pizza.name());
        }
        return pizzaDefined;
    }

    /**
     * Putting the restaurants that has the order pizza in their menu in a HashSet
     * because HashSet deletes duplicates so a valid order have the list of restaurant with one restaurant.
     * @param order order to be validated on whether the pizzas ordered are from different restaurants
     * @return (@boolean true) if restaurants that the order is placed are more than one.
     */
    private boolean multipleRestaurants(Order order){
        Pizza[] orderedPizzas = order.getPizzasInOrder();
        //use Hash Set for a list of restaurant that has the order pizza in its menu.
        //It deletes duplicates and if its length is not 1, it means the order is placed to multiple restaurants.
        HashSet<Restaurant> restaurantsInOrder = new HashSet<>();

        for (Pizza pizza : orderedPizzas) {
            if (menus.containsKey(pizza.name())) {
                restaurantsInOrder.add(menus.get(pizza.name()));
            }
        }

        return restaurantsInOrder.size() > 1;
    }

    /**
     *
     * @param date the order date
     * @param order the order to be validated
     * @param definedRestaurants list of restaurants defined on the REST server
     * @return (@boolean true) if the Day of order date matches one of the opening day of the restaurant.
     */
    private boolean restaurantOpen(LocalDate date, Order order, Restaurant[] definedRestaurants){

        for (int i=0; i < getRestaurant(order, definedRestaurants).openingDays().length; i++){
            //Check if the order day matches with a day in the openingDays
            if(date.getDayOfWeek() == getRestaurant(order, definedRestaurants).openingDays()[i]){
                return true;
            }
        }

        return false;
    }

    /**
     * find the restaurant that the order is placed at
     * get the first pizza in the order and check in which restaurant's menu the pizza is included.
     * @param order order placed in a restaurant
     * @param definedRestaurants restaurants definned on the REST server.
     * @return the restaurant that the order is placed at
     */
    public Restaurant getRestaurant(Order order, Restaurant[] definedRestaurants){
        //go through each given restaurant
        Pizza pizza = order.getPizzasInOrder()[0];
        for (Restaurant r : definedRestaurants) {
            for (int j = 0; j < r.menu().length; j++) {
                //check if the given pizza matches any of the pizzas from the given restaurants
                if (pizza.name() == null ? r.menu()[j].name() == null : pizza.name().equals(r.menu()[j].name())) {
                    return r;
                }
            }
        }

        return null;
    }

    /**
     * the order go through several different checks and assigned with the correlated order status and validation code.
     * @param orderToValidate order to be validated
     * @param definedRestaurants defined restaurant on the REST server
     * @return the order validated with its order status and validation code set.
     */
    @Override
    public Order validateOrder(Order orderToValidate, Restaurant[] definedRestaurants){
        String creditCardNumber = orderToValidate.getCreditCardInformation().getCreditCardNumber();
        String creditCardExpiry = orderToValidate.getCreditCardInformation().getCreditCardExpiry();
        String creditCardCVV = orderToValidate.getCreditCardInformation().getCvv();
        Pizza[] pizzasInOrder = orderToValidate.getPizzasInOrder();
        //CARD_NUMBER_INVALID
        if (creditCardNumber == null || creditCardNumber.length() != 16 || !(creditCardNumber.matches("\\d+"))) { //check if each element are numbers
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.CARD_NUMBER_INVALID);
            return orderToValidate;

        }

        //EXPIRY_DATE_INVALID
        else if (!expiryDateValid(orderToValidate.getCreditCardInformation(), orderToValidate.getOrderDate())){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.EXPIRY_DATE_INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.EXPIRY_DATE_INVALID);
            return orderToValidate;

        }

        //CVV_INVALID
        else if (creditCardCVV==null || creditCardCVV.length() != 3 || !(creditCardCVV.matches("\\d+"))){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.CVV_INVALID);
            return orderToValidate;
        }

        //TOTAL_INCORRECT
        else if (!totalCost(orderToValidate)){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.TOTAL_INCORRECT);
            return orderToValidate;
        }

        //PIZZA_NOT_DEFINED
        else if (!pizzaExists(orderToValidate, definedRestaurants) || getRestaurant(orderToValidate, definedRestaurants) == null){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.PIZZA_NOT_DEFINED);
            return orderToValidate;
        }

        //MAX_PIZZA_COUNT_EXCEED
        else if (pizzasInOrder.length > SystemConstants.MAX_PIZZAS_PER_ORDER) {
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.MAX_PIZZA_COUNT_EXCEEDED);
            return orderToValidate;
        }

        //PIZZA_FROM_MULTIPLE_RESTAURANTS
        else if (multipleRestaurants(orderToValidate)){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.PIZZA_FROM_MULTIPLE_RESTAURANTS);
            return orderToValidate;
        }

        //RESTAURANT_CLOSED
        else if (!multipleRestaurants(orderToValidate) && !(restaurantOpen(orderToValidate.getOrderDate(), orderToValidate, definedRestaurants))){
            orderToValidate.setOrderStatus(OrderStatus.INVALID);
            orderToValidate.setOrderValidationCode(OrderValidationCode.RESTAURANT_CLOSED);
            return orderToValidate;
        }

        //the order is valid, without any error
        else{
            orderToValidate.setOrderStatus(OrderStatus.VALID_BUT_NOT_DELIVERED);
            orderToValidate.setOrderValidationCode(OrderValidationCode.NO_ERROR);
            return orderToValidate;
        }


    }
}
