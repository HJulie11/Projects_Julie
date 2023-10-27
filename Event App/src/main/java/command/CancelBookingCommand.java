package command;

import controller.Context;
import java.time.LocalDateTime;
import java.util.Map;
import model.Booking;
import model.BookingStatus;
import model.Consumer;
import model.Event;
import model.User;
import view.IView;

/**
 * {@link CancelBookingCommand} allows {@link model.Consumer}s to cancel a {@link Booking} given its
 * unique booking number. The command applies for the currently logged-in user.
 */
public class CancelBookingCommand implements ICommand<Boolean> {
  private final long bookingNumber;
  private Boolean successResult;

  /**
   * @param bookingNumber booking number uniquely identifying a {@link Booking} that was previously
   *     made by the currently logged in {@link Consumer}
   */
  public CancelBookingCommand(long bookingNumber) {
    this.bookingNumber = bookingNumber;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that currently logged-in user is a Consumer
   * @verifies.that the booking number corresponds to an existing Booking
   * @verifies.that the logged-in user is the booking owner
   * @verifies.that the booking is still active (i.e., not cancelled previously)
   * @verifies.that the booked performance start is at least 24h away from now
   * @verifies.that if the event was paid, that the refund is successful
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user looged in and consumer
    User currentUser = context.getUserState().getCurrentUser();
    if (!(currentUser instanceof Consumer)) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_USER_NOT_CONSUMER,
          Map.of(
              "bookingNumber",
              bookingNumber,
              "currentUser",
              currentUser != null ? currentUser : "none"));
      successResult = false;
      return;
    }
    Consumer consumer = (Consumer) currentUser;

    // Check booking exists
    Booking booking = context.getBookingState().findBookingByNumber(bookingNumber);
    if (booking == null) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_BOOKING_NOT_FOUND,
          Map.of("bookingNumber", bookingNumber));
      successResult = false;
      return;
    }

    // Check user booked event
    if (consumer != booking.getBooker()) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_USER_IS_NOT_BOOKER,
          Map.of("bookingNumber", bookingNumber));
      successResult = false;
      return;
    }

    // Check booking is active
    if (booking.getStatus() != BookingStatus.Active) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_BOOKING_NOT_ACTIVE,
          Map.of("bookingNumber", bookingNumber));
      successResult = false;
      return;
    }

    // Check event isn't too soon
    Event event = booking.getEvent();
    boolean shouldRefund = LocalDateTime.now().plusHours(24).isBefore(event.getStartDateTime());
    if (!shouldRefund) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_NO_CANCELLATIONS_WITHIN_24H,
          Map.of("bookingNumber", bookingNumber));
      successResult = false;
      return;
    }

    // Refund booking
    boolean refundSucceeded =
        (event.getTicketPriceInPence() <= 0
            || context
                .getPaymentSystem()
                .processRefund(
                    consumer.getEmail(),
                    context.getOrgEmail(),
                    booking.getNumTickets() * event.getTicketPriceInPence()));
    if (!refundSucceeded) {
      view.displayFailure(
          "CancelBookingCommand",
          LogStatus.CANCEL_BOOKING_REFUND_FAILED,
          Map.of("bookingNumber", bookingNumber));
      successResult = false;
      return;
    }

    // Cancel booking
    booking.cancelByConsumer();
    event.setNumTicketsLeft(event.getNumTicketsLeft() + booking.getNumTickets());

    view.displaySuccess(
        "CancelBookingCommand",
        LogStatus.CANCEL_BOOKING_SUCCESS,
        Map.of("bookingNumber", bookingNumber));
    successResult = true;
  }

  /**
   * @return True if successful and false otherwise
   */
  @Override
  public Boolean getResult() {
    return successResult;
  }

  private enum LogStatus {
    CANCEL_BOOKING_SUCCESS,
    CANCEL_BOOKING_USER_NOT_CONSUMER,
    CANCEL_BOOKING_BOOKING_NOT_FOUND,
    CANCEL_BOOKING_USER_IS_NOT_BOOKER,
    CANCEL_BOOKING_BOOKING_NOT_ACTIVE,
    CANCEL_BOOKING_NO_CANCELLATIONS_WITHIN_24H,
    CANCEL_BOOKING_REFUND_FAILED,
  }
}
