package command;

import controller.Context;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import model.Booking;
import model.Consumer;
import model.Event;
import model.Review;
import model.User;
import view.IView;

/**
 * {@link ReviewEventCommand} allows the logged in {@link Consumer} to create a new {@link Review}
 * for a given {@link Event}.
 */
public class ReviewEventCommand implements ICommand<Review> {

  private final long eventNumber;
  private Review reviewResult;
  private String content;

  /**
   * @param eventNumber number of event
   * @param content content of the review
   */
  public ReviewEventCommand(long eventNumber, String content) {
    this.eventNumber = eventNumber;
    this.content = content;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that an event exists with the corresponding eventNumber
   * @verifies.that the event is already over
   * @verifies.that the current user is a logged-in Consumer
   * @verifies.that the consumer had at least 1 valid booking (not cancelled by the consumer) at the
   *     event
   */
  @Override
  public void execute(Context context, IView view) {
    // Check event exists
    Event event = context.getEventState().findEventByNumber(eventNumber);
    if (event == null) {
      view.displayFailure(
          "ReviewEventCommand",
          LogStatus.REVIEW_EVENT_NOT_FOUND,
          Map.of("eventNumber", eventNumber));
      reviewResult = null;
      return;
    }

    // Check event has ended
    if (LocalDateTime.now().isBefore(event.getEndDateTime())) {
      view.displayFailure(
          "ReviewEventCommand",
          LogStatus.REVIEW_EVENT_NOT_OVER,
          Map.of("endDataTime", event.getEndDateTime()));
      reviewResult = null;
      return;
    }

    // Check user logged in and consumer
    User currentUser = context.getUserState().getCurrentUser();
    if (!(currentUser instanceof Consumer)) {
      view.displayFailure(
          "ReviewEventCommand",
          LogStatus.REVIEW_EVENT_USER_NOT_CONSUMER,
          Map.of("currentUser", String.valueOf(currentUser)));
      reviewResult = null;
      return;
    }
    Consumer consumer = (Consumer) currentUser;

    // Check user booked event
    List<Booking> bookings = consumer.getBookings();
    boolean contains = false;
    for (Booking booking : bookings) {
      if (booking.getEvent() == event) {
        contains = true;
      }
    }
    if (!(contains)) {
      view.displayFailure(
          "ReviewEventCommand",
          LogStatus.REVIEW_EVENT_NO_VALID_BOOKING,
          Map.of("bookings", consumer.getBookings()));
      reviewResult = null;
      return;
    }

    // Create review
    Review reviewResult = new Review(consumer, event, LocalDateTime.now(), content);
    event.addReview(reviewResult);
    view.displaySuccess(
        "ReviewEventCommand", LogStatus.REVIEW_EVENT_SUCCESS, Map.of("review", reviewResult));
  }

  /**
   * @return The new {@link Review} if successful and null otherwise
   */
  @Override
  public Review getResult() {
    return reviewResult;
  }

  private enum LogStatus {
    REVIEW_EVENT_SUCCESS,
    REVIEW_EVENT_NOT_FOUND,
    REVIEW_EVENT_NOT_OVER,
    REVIEW_EVENT_USER_NOT_CONSUMER,
    REVIEW_EVENT_NO_VALID_BOOKING
  }
}
