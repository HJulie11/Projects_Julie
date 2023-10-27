package model;

import java.io.Serializable;
import java.time.LocalDateTime;

/** {@link Booking} represents a booking made by a {@link Consumer} for an {@link Event}. */
public class Booking implements Serializable {
  private final long bookingNumber;
  private final Consumer booker;
  private final Event event;
  private final int numTickets;
  private final LocalDateTime bookingDateTime;
  private BookingStatus status;

  /**
   * @param bookingNumber unique identifier for this booking
   * @param booker the {@link Consumer} who made this booking
   * @param event the {@link Event} this booking is for
   * @param numTickets the number of booked tickets
   * @param bookingDateTime the date and time when this booking was made
   */
  public Booking(
      long bookingNumber,
      Consumer booker,
      Event event,
      int numTickets,
      LocalDateTime bookingDateTime) {
    this.status = BookingStatus.Active;
    this.booker = booker;
    this.event = event;
    this.bookingNumber = bookingNumber;
    this.numTickets = numTickets;
    this.bookingDateTime = bookingDateTime;
  }

  /**
   * @param bookingNumber unique identifier for this booking
   * @param booking The {@link Booking} to copy
   */
  public Booking(long bookingNumber, Booking booking) {
    this.bookingNumber = bookingNumber;
    this.booker = booking.booker;
    this.event = booking.event;
    this.numTickets = booking.numTickets;
    this.bookingDateTime = booking.bookingDateTime;
    this.status = booking.status;
  }

  public long getBookingNumber() {
    return bookingNumber;
  }

  public BookingStatus getStatus() {
    return status;
  }

  public Consumer getBooker() {
    return booker;
  }

  public Event getEvent() {
    return event;
  }

  public int getNumTickets() {
    return numTickets;
  }

  public LocalDateTime getBookingDateTime() {
    return bookingDateTime;
  }

  /** Sets the {@link #status} to {@link BookingStatus#CancelledByConsumer}. */
  public void cancelByConsumer() {
    this.status = BookingStatus.CancelledByConsumer;
  }

  /** Sets the {@link #status} to {@link BookingStatus#CancelledByProvider}. */
  public void cancelByProvider() {
    this.status = BookingStatus.CancelledByProvider;
  }

  @Override
  public String toString() {
    return "Booking{"
        + "status="
        + status
        + ", bookingNumber="
        + bookingNumber
        + ", booker="
        + booker.getName()
        + ", event="
        + event
        + ", numTickets="
        + numTickets
        + ", bookingDateTime="
        + bookingDateTime
        + '}';
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof Booking)) {
      return false;
    }
    Booking booking = (Booking) other;
    // Check same number of tickets
    if (numTickets != booking.numTickets) {
      return false;
    }
    // Check same booker
    if (!booker.getEmail().equals(booking.booker.getEmail())) {
      return false;
    }
    // Check same event
    if (!event.equals(booking.event)) {
      return false;
    }
    // Check same booking time
    if (!bookingDateTime.equals(booking.bookingDateTime)) {
      return false;
    }
    // Check same status
    if (!status.equals(booking.status)) {
      return false;
    }
    return true;
  }
}
