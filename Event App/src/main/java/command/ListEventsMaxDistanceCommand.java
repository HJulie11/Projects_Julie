package command;

import com.graphhopper.util.shapes.GHPoint;
import controller.Context;
import external.MapSystem;
import java.time.LocalDate;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import model.Consumer;
import model.Event;
import model.EventStatus;
import model.TransportMode;
import model.User;
import view.IView;

/**
 * {@link ListEventsMaxDistanceCommand} allows a logged in {@link Consumer} to generate a list of
 * events within a given distance.
 */
public class ListEventsMaxDistanceCommand extends ListEventsCommand {
  private final TransportMode transportMode;
  private final double maxDistance;

  /**
   * @param userEventsOnly if true, the returned events will be filtered depending on the logged-in
   *     user: for {@link Staff}s only the {@link Event}s they have created, and for {@link
   *     Consumer}s only the {@link Event}s that match their {@link EventTagCollection}
   * @param activeEventsOnly if true, returned {@link Event}s will be filtered to contain only
   *     {@link Event}s with {@link EventStatus#ACTIVE}
   * @param searchDate chosen date to look for events. Can be null. If not null, only {@link Event}s
   *     that are happening on {@link #searchDate} (i.e., starting, ending, or in between) will be
   *     included
   * @param transportMode mode of transportation to calculate distance for
   * @param maxDistance maximum distance of each {@link Event} from the {@link Consumer}'s address
   */
  public ListEventsMaxDistanceCommand(
      boolean userEventsOnly,
      boolean activeEventsOnly,
      LocalDate searchDate,
      TransportMode transportMode,
      double maxDistance) {
    super(userEventsOnly, activeEventsOnly, searchDate);
    this.transportMode = transportMode;
    this.maxDistance = maxDistance;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that if userEventsOnly is set, the current user must be logged in
   * @verifies.that currently logged-in user is a {@link Consumer}
   * @verifies.that current user has an address set up in their profile
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user logged in and consumer
    User user = context.getUserState().getCurrentUser();
    if (!(user instanceof Consumer)) {
      view.displayFailure(
          "ListEventsMaxDistanceCommand",
          LogStatus.LIST_EVENTS_MAX_DISTANCE_USER_NOT_CONSUMER,
          Map.of("currentUser", user));
      return;
    }

    // Check user has address
    if (((Consumer) user).getAddress() == "") {
      view.displayFailure(
          "ListEventsMaxDistanceCommand",
          LogStatus.LIST_EVENTS_MAX_DISTANCE_USER_HAS_NO_SETUP_ADDRESS,
          Map.of("currentUser", user));
      return;
    }

    // Filter events
    List<Event> events = context.getEventState().getAllEvents();
    MapSystem mapSystem = context.getMapSystem();
    GHPoint userAddress = mapSystem.convertToCoordinates(((Consumer) user).getAddress());
    HashMap<Long, Double> eventDistances = new HashMap<>();
    for (int i = 0; i < events.size(); i++) {
      eventDistances.put(
          events.get(i).getEventNumber(),
          mapSystem
              .routeBetweenPoints(
                  transportMode,
                  userAddress,
                  mapSystem.convertToCoordinates(events.get(i).getVenueAddress()))
              .getDistance());
    }
    List<Event> filteredEvents =
        IntStream.range(0, events.size())
            .filter(
                i ->
                    eventDistances.get(events.get(i).getEventNumber()) <= maxDistance
                        && eventSatisfiesPreferences(
                            context.getEventState().getPossibleTags(),
                            ((Consumer) user).getPreferences(),
                            events.get(i)))
            .mapToObj(i -> events.get(i))
            .collect(Collectors.toList());
    Stream<Event> secondfilteredEvents = filteredEvents.stream();
    if (activeEventsOnly) {
      secondfilteredEvents =
          secondfilteredEvents.filter(event -> event.getStatus() == EventStatus.ACTIVE);
    }
    if (searchDate != null) {
      secondfilteredEvents =
          secondfilteredEvents.filter(
              event ->
                  event.getStartDateTime().toLocalDate().equals(searchDate)
                      || event.getEndDateTime().toLocalDate().equals(searchDate)
                      || (searchDate.isAfter(event.getStartDateTime().toLocalDate())
                          && searchDate.isBefore(event.getEndDateTime().toLocalDate())));
    }
    filteredEvents = secondfilteredEvents.collect(Collectors.toList());

    // Sort events by distance
    Collections.sort(
        filteredEvents, Comparator.comparing(e -> eventDistances.get(e.getEventNumber())));

    // List events
    eventListResult = filteredEvents;
    view.displaySuccess("ListEventsMaxDistanceCommand", LogStatus.LIST_EVENTS_MAX_DISTANCE_SUCCESS);
  }

  private enum LogStatus {
    LIST_EVENTS_MAX_DISTANCE_SUCCESS,
    LIST_EVENTS_MAX_DISTANCE_USER_NOT_CONSUMER,
    LIST_EVENTS_MAX_DISTANCE_USER_HAS_NO_SETUP_ADDRESS
  }
}
