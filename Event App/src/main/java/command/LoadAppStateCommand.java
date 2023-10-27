package command;

import controller.Context;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import model.Booking;
import model.Event;
import model.EventTag;
import model.Staff;
import model.User;
import view.IView;

/**
 * {@link LoadAppStateCommand} allows the logged in {@link Staff} member to import a previously
 * saved app state.
 *
 * <p>{@link LoadAppStateCommand} extends the existing in-memory state. Organisation details (name,
 * address, email address, secret) and external system references are ignored. The current user is
 * not replaced. If there are no clashes, identifiers (like event numbers or booking numbers) are
 * renumbered for the newly loaded items to prevent conflicts.
 */
public class LoadAppStateCommand implements ICommand<Boolean> {
  private Boolean successResult;
  private String filename;

  /**
   * @param filename The path to the file to save the data to.
   */
  public LoadAppStateCommand(String filename) {
    if (!filename.endsWith(".dat")) {
      filename += ".dat";
    }
    this.filename = filename;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that the currently logged-in user is a Staff member
   * @verifies.that For event tags, clashing tags (with the same name but different values) cause
   *     the whole operation to be cancelled. This is because replacing or ignoring tags could mess
   *     up Events or Consumers using either old or new tags.
   * @verifies.that For users, any user email clashes for users that are not the same result in the
   *     operation being cancelled. This is because overwriting or omitting users could break saved
   *     Bookings.
   * @verifies.that For events, clashing events (with the same title, startDateTime, and
   *     endDateTime) that are not identical cause the operation to be cancelled because otherwise
   *     this could result in either duplicate events or issues with connected bookings.
   * @verifies.that For bookings, clashing bookings (by the same Consumer for the same Events with
   *     the same bookingDateTime) cause the operation to be cancelled because otherwise this could
   *     result in duplicate bookings.
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user logged in and staff
    User currentUser = context.getUserState().getCurrentUser();
    if (currentUser == null) {
      view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_NOT_LOGGED_IN);
      successResult = false;
      return;
    }
    if (!(currentUser instanceof Staff)) {
      view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_LOGGED_IN_USER_NOT_STAFF);
      successResult = false;
      return;
    }

    // Read saved file
    Context savedContext = null;
    try (FileInputStream fis = new FileInputStream(filename);
        ObjectInputStream in = new ObjectInputStream(fis)) {
      savedContext = (Context) in.readObject();
    } catch (Exception exception) {
      view.displayFailure(
          "LoadAppStateCommand",
          LogStatus.LOAD_APP_STATE_COULD_NOT_READ_FILE,
          Map.of("exception", exception.getMessage()));
      successResult = false;
      return;
    }

    // Check for tag clashes
    Map<String, EventTag> savedTags = savedContext.getEventState().getPossibleTags();
    Map<String, EventTag> currentTags = context.getEventState().getPossibleTags();
    for (String tag : savedTags.keySet()) {
      EventTag matchingTag = currentTags.get(tag);
      if (matchingTag != null && !savedTags.get(tag).equals(matchingTag)) {
        view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_TAGS_MISMATCH);
        successResult = false;
        return;
      }
    }

    // Check for event clashes
    List<Event> savedEvents = savedContext.getEventState().getAllEvents();
    List<Event> currentEvents = context.getEventState().getAllEvents();
    for (Event event : savedEvents) {
      List<Event> filteredEvents =
          currentEvents.stream()
              .filter(
                  (Event ev) ->
                      ev.getTitle().equals(event.getTitle())
                          && ev.getStartDateTime().equals(event.getStartDateTime())
                          && ev.getEndDateTime().equals(event.getEndDateTime()))
              .collect(Collectors.toList());
      Event matchingEvent = filteredEvents.size() > 0 ? filteredEvents.get(0) : null;
      if (matchingEvent != null && !event.equals(matchingEvent)) {
        view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_EVENTS_MISMATCH);
        successResult = false;
        return;
      }
    }

    // Check for booking clashes
    List<Booking> savedBookings = savedContext.getBookingState().getAllBookings();
    List<Booking> currentBookings = context.getBookingState().getAllBookings();
    for (Booking booking : savedBookings) {
      List<Booking> filteredBookings =
          currentBookings.stream()
              .filter(
                  (Booking bk) ->
                      bk.getBooker().equals(booking.getBooker())
                          && bk.getEvent().equals(booking.getEvent())
                          && bk.getBookingDateTime().equals(booking.getBookingDateTime()))
              .collect(Collectors.toList());
      Booking matchingBooking = filteredBookings.size() > 0 ? filteredBookings.get(0) : null;
      if (matchingBooking != null && !booking.equals(matchingBooking)) {
        view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_BOOKINGS_MISMATCH);
        successResult = false;
        return;
      }
    }

    // Check for user clashes
    Map<String, User> savedUsers = savedContext.getUserState().getAllUsers();
    Map<String, User> currentUsers = context.getUserState().getAllUsers();
    for (User user : savedUsers.values()) {
      User matchingUser = currentUsers.get(user.getEmail());
      if (matchingUser != null && !user.equals(matchingUser)) {
        view.displayFailure("LoadAppStateCommand", LogStatus.LOAD_APP_STATE_USERS_MISMATCH);
        successResult = false;
        return;
      }
    }

    // Add tags
    for (String tag : savedTags.keySet()) {
      context
          .getEventState()
          .createEventTag(tag, savedTags.get(tag).values, savedTags.get(tag).defaultValue);
    }

    // Add events
    for (Event event : savedEvents) {
      context.getEventState().addEvent(event);
    }

    // Add bookings
    for (Booking booking : savedBookings) {
      context.getBookingState().addBooking(booking);
    }

    // Add users
    for (User user : savedUsers.values()) {
      context.getUserState().addUser(user);
    }

    successResult = true;
    view.displaySuccess("LoadAppStateCommand", successResult);
    return;
  }

  /**
   * @return true if successful and false otherwise
   */
  @Override
  public Boolean getResult() {
    return successResult;
  }

  private enum LogStatus {
    LOAD_APP_STATE_SUCCESS,
    LOAD_APP_STATE_COULD_NOT_READ_FILE,
    LOAD_APP_STATE_NOT_LOGGED_IN,
    LOAD_APP_STATE_LOGGED_IN_USER_NOT_STAFF,
    LOAD_APP_STATE_TAGS_MISMATCH,
    LOAD_APP_STATE_USERS_MISMATCH,
    LOAD_APP_STATE_EVENTS_MISMATCH,
    LOAD_APP_STATE_BOOKINGS_MISMATCH
  }
}
