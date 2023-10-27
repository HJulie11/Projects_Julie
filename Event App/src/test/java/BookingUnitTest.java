import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.LocalDateTime;
import model.Booking;
import model.BookingStatus;
import model.Consumer;
import model.Event;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class BookingUnitTest {
  @Test
  void cancelByConsumerTest() {
    Booking booking =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    booking.cancelByConsumer();
    assertEquals(BookingStatus.CancelledByConsumer, booking.getStatus());
  }

  @Test
  void cancelByProviderTest() {
    Booking booking =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    booking.cancelByProvider();
    assertEquals(BookingStatus.CancelledByProvider, booking.getStatus());
  }

  @Test
  void toStringTest() {
    Booking booking =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertEquals(
        "Booking{"
            + "status=Active"
            + ", bookingNumber=42"
            + ", booker=Alan Turing"
            + ", event=Event{eventNumber=42, title='The Immitation Game', "
            + "type=Movie, numTicketsCap=50, ticketPriceInPence=0, "
            + "venueAddress='51.998 0.741', description='Viewing of 'The "
            + "Immitaiton Game' at Bletchley Park.', "
            + "startDateTime=2023-09-30T07:30, endDateTime=2023-09-30T10:30, "
            + "tags=EventTagCollection{Pets Allowed=No,Kids Allowed=Absolutely not}, "
            + "status=ACTIVE, numTicketsLeft=50}"
            + ", numTickets=10"
            + ", bookingDateTime=2023-08-30T10:30"
            + "}",
        booking.toString());
  }

  @Test
  void equalsSameObjectTest() {
    Booking booking =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertTrue(booking.equals(booking));
  }

  @Test
  void equalsSameArgsTest() {
    Booking booking1 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    Booking booking2 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertTrue(booking1.equals(booking2));
  }

  @Test
  void equalsDifferentConsumerTest() {
    Booking booking1 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    Booking booking2 =
        new Booking(
            42,
            new Consumer(
                "Ada Lovelace",
                "ada.lovelace@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertFalse(booking1.equals(booking2));
  }

  @Test
  void equalsDifferentEventTest() {
    Booking booking1 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    Booking booking2 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "Star Wars: The Empire Strikes Back",
                EventType.Movie,
                50,
                0,
                "38.0594 122.6544",
                "Viewing of 'Star Wars: The Empire Strikes Back' at Skywalker Ranch.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertFalse(booking1.equals(booking2));
  }

  @Test
  void equalsDifferentNumTicketsTest() {
    Booking booking1 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    Booking booking2 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            5,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    assertFalse(booking1.equals(booking2));
  }

  @Test
  void equalsDifferentBookingTimeTest() {
    Booking booking1 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2023, 8, 30, 10, 30, 0));
    Booking booking2 =
        new Booking(
            42,
            new Consumer(
                "Alan Turing",
                "alan.turing@bletchleypark.com",
                "01908 640404",
                "51.998 0.741",
                "VonNeumann123"),
            new Event(
                42,
                "The Immitation Game",
                EventType.Movie,
                50,
                0,
                "51.998 0.741",
                "Viewing of 'The Immitaiton Game' at Bletchley Park.",
                LocalDateTime.of(2023, 9, 30, 7, 30, 0),
                LocalDateTime.of(2023, 9, 30, 10, 30, 0),
                new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not")),
            10,
            LocalDateTime.of(2022, 8, 30, 10, 30, 0));
    assertFalse(booking1.equals(booking2));
  }
}
