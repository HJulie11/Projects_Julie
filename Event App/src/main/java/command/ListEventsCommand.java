package command;

import controller.Context;
import java.time.LocalDate;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import model.Consumer;
import model.Event;
import model.EventStatus;
import model.EventTag;
import model.EventTagCollection;
import model.Staff;
import model.User;
import view.IView;

/**
 * {@link ListEventsCommand} allows anyone to get a list of {@link Event}s available on the system.
 * Optionally, users can specify a particular {@link LocalDate} to look up events for.
 */
public class ListEventsCommand implements ICommand<List<Event>> {
  protected final boolean userEventsOnly;
  protected final boolean activeEventsOnly;
  protected final LocalDate searchDate;
  protected List<Event> eventListResult;

  /**
   * @param userEventsOnly if true, the returned events will be filtered depending on the logged-in
   *     user: for {@link Staff}s only the {@link Event}s they have created, and for {@link
   *     Consumer}s only the {@link Event}s that match their {@link EventTagCollection}
   * @param activeEventsOnly if true, returned {@link Event}s will be filtered to contain only
   *     {@link Event}s with {@link EventStatus#ACTIVE}
   * @param searchDate chosen date to look for events. Can be null. If not null, only {@link Event}s
   *     that are happening on {@link #searchDate} (i.e., starting, ending, or in between) will be
   *     included
   */
  public ListEventsCommand(boolean userEventsOnly, boolean activeEventsOnly, LocalDate searchDate) {
    this.userEventsOnly = userEventsOnly;
    this.activeEventsOnly = activeEventsOnly;
    this.searchDate = searchDate;
  }

  /**
   * Checks event statisfies user preferences
   *
   * @param possibleTags tags for event
   * @param preferences preferences for user
   * @param event event to check
   * @return true if the event satisfies the preferences, false otherwise
   */
  public static boolean eventSatisfiesPreferences(
      Map<String, EventTag> possibleTags, EventTagCollection preferences, Event event) {
    EventTagCollection eventTags = event.getTags();
    for (String key : possibleTags.keySet()) {
      if (preferences.getTags().containsKey(key)) {
        if (!eventTags.getTags().containsKey(key)) {
          if (!(preferences.getValueFor(key).equals(possibleTags.get(key).defaultValue))) {
            return false;
          }
        } else {
          if (!(preferences.getValueFor(key).equals(eventTags.getValueFor(key)))) {
            return false;
          }
        }
      }
    }
    return true;
  }

  /**
   * Filters a list off events.
   *
   * @param events list of events to filter
   * @param activeEventsOnly whether to only include active events
   * @param searchDate the date of the events
   * @return A filtered list of events
   */
  private static List<Event> filterEvents(
      List<Event> events, boolean activeEventsOnly, LocalDate searchDate) {
    Stream<Event> filteredEvents = events.stream();
    if (activeEventsOnly) {
      filteredEvents = filteredEvents.filter(event -> event.getStatus() == EventStatus.ACTIVE);
    }
    if (searchDate != null) {
      filteredEvents =
          filteredEvents.filter(
              event ->
                  event.getStartDateTime().toLocalDate().equals(searchDate)
                      || event.getEndDateTime().toLocalDate().equals(searchDate)
                      || (searchDate.isAfter(event.getStartDateTime().toLocalDate())
                          && searchDate.isBefore(event.getEndDateTime().toLocalDate())));
    }
    return filteredEvents.collect(Collectors.toList());
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that if userEventsOnly is set, the current user must be logged in
   */
  @Override
  public void execute(Context context, IView view) {
    // Get list of all events
    if (!userEventsOnly) {
      eventListResult =
          filterEvents(context.getEventState().getAllEvents(), activeEventsOnly, searchDate);
      view.displaySuccess(
          "ListEventsCommand",
          LogStatus.LIST_EVENTS_SUCCESS,
          Map.of(
              "activeEventsOnly",
              activeEventsOnly,
              "userEventsOnly",
              false,
              "searchDate",
              String.valueOf(searchDate),
              "eventList",
              eventListResult));
      return;
    }

    // Check user logged in
    User currentUser = context.getUserState().getCurrentUser();
    if (currentUser == null) {
      view.displayFailure(
          "ListEventsCommand",
          LogStatus.LIST_EVENTS_NOT_LOGGED_IN,
          Map.of("activeEventsOnly", activeEventsOnly, "userEventsOnly", true));
      eventListResult = null;
      return;
    }

    // Check user is staff then list all events
    if (currentUser instanceof Staff) {
      eventListResult =
          filterEvents(context.getEventState().getAllEvents(), activeEventsOnly, searchDate);
      view.displaySuccess(
          "ListEventsCommand",
          LogStatus.LIST_EVENTS_SUCCESS,
          Map.of(
              "activeEventsOnly",
              activeEventsOnly,
              "userEventsOnly",
              true,
              "searchDate",
              String.valueOf(searchDate),
              "eventList",
              eventListResult));
      return;
    }

    // Check user is consumer then list user's events
    if (currentUser instanceof Consumer) {
      HashMap<String, EventTag> possibleTags = context.getEventState().getPossibleTags();
      Consumer consumer = (Consumer) currentUser;
      EventTagCollection preferences = consumer.getPreferences();
      List<Event> eventsFittingPreferences =
          context.getEventState().getAllEvents().stream()
              .filter(event -> eventSatisfiesPreferences(possibleTags, preferences, event))
              .collect(Collectors.toList());

      eventListResult = filterEvents(eventsFittingPreferences, activeEventsOnly, searchDate);
      view.displaySuccess(
          "ListEventsCommand",
          LogStatus.LIST_EVENTS_SUCCESS,
          Map.of(
              "activeEventsOnly",
              activeEventsOnly,
              "userEventsOnly",
              true,
              "searchDate",
              String.valueOf(searchDate),
              "eventList",
              eventListResult));
      return;
    }

    eventListResult = null;
  }

  /**
   * @return List of {@link Event}s if successful and null otherwise
   */
  @Override
  public List<Event> getResult() {
    return eventListResult;
  }

  private enum LogStatus {
    LIST_EVENTS_SUCCESS,
    LIST_EVENTS_NOT_LOGGED_IN,
  }
}
