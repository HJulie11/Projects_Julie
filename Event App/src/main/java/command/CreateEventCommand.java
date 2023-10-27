package command;

import com.graphhopper.util.shapes.GHPoint;
import controller.Context;
import external.MapSystem;
import java.time.LocalDateTime;
import java.util.Map;
import model.Event;
import model.EventTagCollection;
import model.EventType;
import model.Staff;
import model.User;
import view.IView;

/**
 * {@link CreateEventCommand} allows {@link Staff}s to create new {@link Event}s. The command
 * applies for the currently logged-in user.
 */
public class CreateEventCommand implements ICommand<Event> {
  private final String title;
  private final EventType type;
  private final int numTickets;
  private final int ticketPriceInPence;
  private final String venueAddress;
  private final String description;
  private final LocalDateTime startDateTime;
  private final LocalDateTime endDateTime;
  private final EventTagCollection tags;
  private Event eventResult;

  /**
   * @param title title of the event
   * @param type type of the event
   * @param numTickets number of initially available tickets for the event. This can be 0 if the
   *     event does not need booking.
   * @param ticketPriceInPence price in GBP pence per event ticket. This can be 0 if the event is
   *     free.
   * @param venueAddress indicates where this performance will take place, would be displayed to
   *     users in app
   * @param description additional details about the event
   * @param startDateTime indicates the date and time when this performance is due to start
   * @param endDateTime indicates the date and time when this performance is due to end
   */
  public CreateEventCommand(
      String title,
      EventType type,
      int numTickets,
      int ticketPriceInPence,
      String venueAddress,
      String description,
      LocalDateTime startDateTime,
      LocalDateTime endDateTime,
      EventTagCollection tags) {
    this.title = title;
    this.type = type;
    this.numTickets = numTickets;
    this.ticketPriceInPence = ticketPriceInPence;
    this.venueAddress = venueAddress;
    this.description = description;
    this.startDateTime = startDateTime;
    this.endDateTime = endDateTime;
    this.tags = tags;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that currently logged-in user is a Staff member
   * @verifies.that event startDateTime is not after endDateTime
   * @verifies.that event startDateTime is in the future
   * @verifies.that no other event with the same title has the same startDateTime and endDateTime
   * @verifies.that the event ticket price is non-negative
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user logged in and staff
    User currentUser = context.getUserState().getCurrentUser();
    if (!(currentUser instanceof Staff)) {
      view.displayFailure(
          "CreateEventCommand",
          CreateEventCommand.LogStatus.CREATE_EVENT_USER_NOT_STAFF,
          Map.of("user", currentUser != null ? currentUser : "none"));
      eventResult = null;
      return;
    }

    // Check event end date is after start date
    if (startDateTime.isAfter(endDateTime)) {
      view.displayFailure(
          "CreateEventCommand",
          LogStatus.CREATE_EVENT_START_AFTER_END,
          Map.of("startDateTime", startDateTime, "endDateTime", endDateTime));
      eventResult = null;
      return;
    }

    // Check event start date is in future
    if (startDateTime.isBefore(LocalDateTime.now())) {
      view.displayFailure(
          "CreateEventCommand",
          LogStatus.CREATE_EVENT_IN_THE_PAST,
          Map.of("startDateTime", startDateTime));
      eventResult = null;
      return;
    }

    // Check event doesn't clash with another
    boolean isEventTitleAndTimeClash =
        context.getEventState().getAllEvents().stream()
            .anyMatch(
                otherEvent ->
                    otherEvent.getTitle().equals(title)
                        && otherEvent.getStartDateTime().equals(startDateTime)
                        && otherEvent.getEndDateTime().equals(endDateTime));
    if (isEventTitleAndTimeClash) {
      view.displayFailure(
          "CreateEventCommand",
          LogStatus.CREATE_EVENT_TITLE_AND_TIME_CLASH,
          Map.of("title", title, "startDateTime", startDateTime, "endDateTime", endDateTime));
      eventResult = null;
      return;
    }

    // Check tickets dont't cost less than 0p
    if (ticketPriceInPence < 0) {
      view.displayFailure(
          "CreateEventCommand",
          LogStatus.CREATE_EVENT_NEGATIVE_TICKET_PRICE,
          Map.of("ticketPriceInPence", ticketPriceInPence));
      eventResult = null;
      return;
    }
    // Check address is valid lat-long format and falls within map boundary
    if (!(venueAddress == "")) {
      MapSystem mapSystem = context.getMapSystem();
      if (!venueAddress.matches(MapSystem.validCoordString)) {
        view.displayFailure(
            "CreateEventCommand",
            LogStatus.CREATE_EVENT_ADDRESS_NOT_IN_CORRECT_FORMAT,
            Map.of("venueAddress", venueAddress));
        eventResult = null;
        return;
      }

      GHPoint coords = mapSystem.convertToCoordinates(venueAddress);
      if (mapSystem.isPointWithinMapBounds(coords) == false) {
        // should implement isCoordinate(String) Boolean method to check if the
        view.displayFailure(
            "CreateEventCommand",
            LogStatus.CREATE_EVENT_VENUE_NOT_IN_BOUNDARY,
            Map.of("venueAddress", venueAddress));
        eventResult = null;
        return;
      }
    }

    // Check if event tags provided has known names and values
    boolean isTagNameAndValueKnown =
        tags.getTags().size() == 0
            || (context.getEventState().getPossibleTags().keySet().stream()
                .anyMatch(
                    key ->
                        context.getEventState().getPossibleTags().containsKey(key)
                            && context.getEventState().getPossibleTags().values().stream()
                                .anyMatch(
                                    value ->
                                        context
                                            .getEventState()
                                            .getPossibleTags()
                                            .containsValue(value))));
    if (!isTagNameAndValueKnown) {
      view.displayFailure(
          "CreateEventCommand", LogStatus.CREATE_EVENT_TAG_NOT_KNOWN, Map.of("tags", tags));
      eventResult = null;
      return;
    }

    // Create event
    Event event =
        context
            .getEventState()
            .createEvent(
                title,
                type,
                numTickets,
                ticketPriceInPence,
                venueAddress,
                description,
                startDateTime,
                endDateTime,
                tags);
    view.displaySuccess(
        "CreateEventCommand",
        LogStatus.CREATE_EVENT_SUCCESS,
        Map.of("eventNumber", event.getEventNumber(), "organiser", currentUser, "title", title));
    eventResult = event;
  }

  /**
   * @return event number corresponding to the created event if successful and null otherwise
   */
  @Override
  public Event getResult() {
    return eventResult;
  }

  private enum LogStatus {
    CREATE_EVENT_USER_NOT_STAFF,
    CREATE_EVENT_START_AFTER_END,
    CREATE_EVENT_IN_THE_PAST,
    CREATE_EVENT_TITLE_AND_TIME_CLASH,
    CREATE_EVENT_NEGATIVE_TICKET_PRICE,
    CREATE_EVENT_VENUE_NOT_IN_BOUNDARY,
    CREATE_EVENT_ADDRESS_NOT_IN_CORRECT_FORMAT,
    CREATE_EVENT_TAG_NOT_KNOWN,
    CREATE_EVENT_SUCCESS,
  }
}
