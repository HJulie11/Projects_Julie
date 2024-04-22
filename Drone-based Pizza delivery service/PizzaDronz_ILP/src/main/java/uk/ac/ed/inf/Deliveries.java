package uk.ac.ed.inf;

import uk.ac.ed.inf.ilp.constant.OrderStatus;
import uk.ac.ed.inf.ilp.constant.OrderValidationCode;

/*
 a class to store all the orders that are delivered and not delivered
  */
public class Deliveries {
    private String orderNo;
    private OrderStatus orderStatus;
    private OrderValidationCode orderValidationCode;
    private int costInPence;

    /**
     * The constructor of Deliveries objects with parameters:
     * @param orderNo The order number
     * @param orderStatus The status of the order
     * @param orderValidationCode The validation code of the order
     * @param costInPence the total cost of order in pence.
     */
    public Deliveries(String orderNo, OrderStatus orderStatus, OrderValidationCode orderValidationCode, int costInPence){
        this.orderNo = orderNo;
        this.orderStatus = orderStatus;
        this.orderValidationCode = orderValidationCode;
        this.costInPence = costInPence;
    }

    // Getters for attributes so that the object mapper can get and read the data from the object and put into an outputfile.
    public String getOrderNo() {
        return orderNo;
    }

    public OrderStatus getOrderStatus() {
        return orderStatus;
    }

    public OrderValidationCode getOrderValidationCode() {
        return orderValidationCode;
    }

    public int getCostInPence() {
        return costInPence;
    }

}
