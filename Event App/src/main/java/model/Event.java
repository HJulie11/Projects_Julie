package model;

import java.io.Serializable;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * {@link Event} represents an event that can be booked by {@link Consumer}s. Tickets can be free,
 * but they are required to attend, and there is a maximum cap on the number of tickets that can be
 * booked.
 */
public class Event implements Serializable {
  private final long eventNumber;
  private final String title;
  private final EventType type;
  private final int numTicketsCap;
  private final int ticketPriceInPence;
  private final String venueAddress;
  private final String description;
  private final LocalDateTime startDateTime;
  private final LocalDateTime endDateTime;
  private final EventTagCollection tags;
  private EventStatus status;
  private int numTicketsLeft;
  private List<Review> reviews;

  /**
   * Create a new Event with status = {@link EventStatus#ACTIVE}
   *
   * @param eventNumber unique event identifier
   * @param title name of the event
   * @param type type of the event
   * @param numTicketsCap maximum number of tickets, initially all available for booking
   * @param ticketPriceInPence price of each ticket in GBP pence
   * @param venueAddress address where the performance will be taking place
   * @param description additional details about the event, e.g., who the performers in a concert
   *     will be or if payment is required on entry in addition to ticket booking
   * @param startDateTime date and time when the performance will begin
   * @param endDateTime date and time when the performance will end
   * @param tags the event tags
   */
  public Event(
      long eventNumber,
      String title,
      EventType type,
      int numTicketsCap,
      int ticketPriceInPence,
      String venueAddress,
      String description,
      LocalDateTime startDateTime,
      LocalDateTime endDateTime,
      EventTagCollection tags) {
    this.eventNumber = eventNumber;
    this.title = title;
    this.type = type;
    this.numTicketsCap = numTicketsCap;
    this.ticketPriceInPence = ticketPriceInPence;
    this.venueAddress = venueAddress;
    this.description = description;
    this.startDateTime = startDateTime;
    this.endDateTime = endDateTime;
    this.tags = tags;

    this.status = EventStatus.ACTIVE;
    this.numTicketsLeft = numTicketsCap;
    this.reviews = new ArrayList<>();
  }

  /**
   * @param eventNumber unique identifier for this event
   * @param event The {@link Event} to copy
   */
  public Event(long eventNumber, Event event) {
    this.eventNumber = eventNumber;
    this.title = event.title;
    this.type = event.type;
    this.numTicketsCap = event.numTicketsCap;
    this.ticketPriceInPence = event.ticketPriceInPence;
    this.venueAddress = event.venueAddress;
    this.description = event.description;
    this.startDateTime = event.startDateTime;
    this.endDateTime = event.endDateTime;
    this.tags = event.tags;
    this.status = event.status;
    this.numTicketsLeft = event.numTicketsLeft;
  }

  public void addReview(Review review) {
    reviews.add(review);
  }

  /**
   * @return Number of the maximum cap of tickets which were initially available
   */
  public int getNumTicketsCap() {
    return numTicketsCap;
  }

  public int getNumTicketsLeft() {
    return numTicketsLeft;
  }

  public void setNumTicketsLeft(int numTicketsLeft) {
    this.numTicketsLeft = numTicketsLeft;
  }

  public int getTicketPriceInPence() {
    return ticketPriceInPence;
  }

  public String getVenueAddress() {
    return venueAddress;
  }

  public String getDescription() {
    return description;
  }

  public long getEventNumber() {
    return eventNumber;
  }

  public String getTitle() {
    return title;
  }

  public EventType getType() {
    return type;
  }

  public EventStatus getStatus() {
    return status;
  }

  public LocalDateTime getStartDateTime() {
    return startDateTime;
  }

  public LocalDateTime getEndDateTime() {
    return endDateTime;
  }

  public EventTagCollection getTags() {
    return tags;
  }

  public List<Review> getReviews() {
    return reviews;
  }

  /** Set {@link #status} to {@link EventStatus#CANCELLED} */
  public void cancel() {
    status = EventStatus.CANCELLED;
  }

  @Override
  public String toString() {
    return "Event{"
        + "eventNumber="
        + eventNumber
        + ", title='"
        + title
        + '\''
        + ", type="
        + type
        + ", numTicketsCap="
        + numTicketsCap
        + ", ticketPriceInPence="
        + ticketPriceInPence
        + ", venueAddress='"
        + venueAddress
        + '\''
        + ", description='"
        + description
        + '\''
        + ", startDateTime="
        + startDateTime
        + ", endDateTime="
        + endDateTime
        + ", tags="
        + tags
        + ", status="
        + status
        + ", numTicketsLeft="
        + numTicketsLeft
        + '}';
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof Event)) {
      return false;
    }
    Event event = (Event) other;
    // Check same ticket cap
    if (numTicketsCap != event.numTicketsCap) {
      return false;
    }
    // Check same ticket price
    if (ticketPriceInPence != event.ticketPriceInPence) {
      return false;
    }
    // Check same tickets left
    if (numTicketsLeft != event.numTicketsLeft) {
      return false;
    }
    // Check same title
    if (!title.equals(event.title)) {
      return false;
    }
    // Check same address
    if (!venueAddress.equals(event.venueAddress)) {
      return false;
    }
    // Check same description
    if (!description.equals(event.description)) {
      return false;
    }
    // Check same start time
    if (!startDateTime.equals(event.startDateTime)) {
      return false;
    }
    // Check same end time
    if (!endDateTime.equals(event.endDateTime)) {
      return false;
    }
    // Check same type
    if (!type.equals(event.type)) {
      return false;
    }
    // Check same tags
    if (!tags.equals(event.tags)) {
      return false;
    }
    // Check same status
    if (!status.equals(event.status)) {
      return false;
    }
    // Check same reviews
    if (reviews.size() != event.reviews.size()) {
      return false;
    }
    for (int i = 0; i < reviews.size(); i++) {
      if (!reviews.get(i).equals(event.reviews.get(i))) {
        return false;
      }
    }
    return true;
  }
}
