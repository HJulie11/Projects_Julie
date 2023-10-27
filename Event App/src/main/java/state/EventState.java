package state;

import java.io.Serializable;
import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import model.Event;
import model.EventTag;
import model.EventTagCollection;
import model.EventType;

/** {@link EventState} is a concrete implementation of {@link IEventState}. */
public class EventState implements IEventState, Serializable {
  private final List<Event> events;
  private long nextEventNumber;
  private final HashMap<String, EventTag> possibleTags;

  /**
   * Create a new EventState with an empty list of events, which keeps track of the next event and
   * performance numbers it will generate, starting from 1 and incrementing by 1 each time when
   * requested
   */
  public EventState() {
    events = new LinkedList<>();
    nextEventNumber = 1;
    possibleTags = new HashMap<>();
    EventTag socialDistancing =
        new EventTag(new HashSet<>(Arrays.asList("true", "false")), "false");
    EventTag hasAirFiltration = new EventTag(new HashSet<>(Arrays.asList("true", "No")), "false");
    EventTag venueCapacity =
        new EventTag(new HashSet<>(Arrays.asList("<20", "20-100", "100-200", "200")), "<20");
    possibleTags.put("Has Social Distancing", socialDistancing);
    possibleTags.put("Has Air Filtration", hasAirFiltration);
    possibleTags.put("Venue Capacity", venueCapacity);
  }

  /**
   * Copy constructor to make a deep copy of another EventState instance
   *
   * @param other instance to copy
   */
  public EventState(IEventState other) {
    EventState otherImpl = (EventState) other;
    events = new LinkedList<>(otherImpl.events);
    nextEventNumber = otherImpl.nextEventNumber;
    possibleTags = new HashMap<>(otherImpl.possibleTags);
  }

  @Override
  public List<Event> getAllEvents() {
    return events;
  }

  @Override
  public Event findEventByNumber(long eventNumber) {
    return events.stream()
        .filter(event -> event.getEventNumber() == eventNumber)
        .findFirst()
        .orElse(null);
  }

  @Override
  public Event createEvent(
      String title,
      EventType type,
      int numTickets,
      int ticketPriceInPence,
      String venueAddress,
      String description,
      LocalDateTime startDateTime,
      LocalDateTime endDateTime,
      EventTagCollection tags) {
    long eventNumber = nextEventNumber;
    nextEventNumber++;

    Event event =
        new Event(
            eventNumber,
            title,
            type,
            numTickets,
            ticketPriceInPence,
            venueAddress,
            description,
            startDateTime,
            endDateTime,
            tags);
    addEvent(event);
    return event;
  }

  @Override
  public void addEvent(Event event) {
    events.add(event);
  }

  @Override
  public HashMap<String, EventTag> getPossibleTags() {
    return possibleTags;
  }

  @Override
  public EventTag createEventTag(String tagName, Set<String> values, String defaultValue) {
    EventTag tag = new EventTag(values, defaultValue);
    possibleTags.put(tagName, tag);
    return tag;
  }
}
