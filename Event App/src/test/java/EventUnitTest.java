import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.LocalDateTime;
import model.Consumer;
import model.Event;
import model.EventStatus;
import model.EventTagCollection;
import model.EventType;
import model.Review;
import org.junit.jupiter.api.Test;

public class EventUnitTest {
  @Test
  void addReviewTest() {
    Event event =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Review review =
        new Review(
            new Consumer("name", "email", "phone number", "address", "password"),
            event,
            LocalDateTime.of(2023, 11, 30, 7, 30, 0),
            "This is a review.");
    event.addReview(review);
    assertEquals(1, event.getReviews().size());
    assertTrue(review == event.getReviews().get(0));
  }

  @Test
  void addReviewMultipleTest() {
    Event event =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Review review1 =
        new Review(
            new Consumer("name", "email", "phone number", "address", "password"),
            event,
            LocalDateTime.of(2023, 11, 30, 7, 30, 0),
            "This is a review.");
    Review review2 =
        new Review(
            new Consumer("name", "email", "phone number", "address", "password"),
            event,
            LocalDateTime.of(2023, 11, 30, 7, 30, 0),
            "This is a review.");
    event.addReview(review1);
    event.addReview(review2);
    assertEquals(2, event.getReviews().size());
    assertTrue(review1 == event.getReviews().get(0));
    assertTrue(review2 == event.getReviews().get(1));
  }

  @Test
  void cancelTest() {
    Event event =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    event.cancel();
    assertEquals(EventStatus.CANCELLED, event.getStatus());
  }

  @Test
  void toStringTest() {
    Event event =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertEquals(
        "Event{"
            + "eventNumber=42"
            + ", title='The Immitation Game'"
            + ", type=Movie"
            + ", numTicketsCap=50"
            + ", ticketPriceInPence=0"
            + ", venueAddress='51.998 0.741'"
            + ", description='Viewing of 'The Immitaiton Game' at Bletchley Park.'"
            + ", startDateTime=2023-09-30T07:30"
            + ", endDateTime=2023-09-30T10:30"
            + ", tags=EventTagCollection{Pets Allowed=No,Kids Allowed=Absolutely not}"
            + ", status=ACTIVE"
            + ", numTicketsLeft=50"
            + "}",
        event.toString());
  }

  @Test
  void equalsSameObjectTest() {
    Event event =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertTrue(event.equals(event));
  }

  @Test
  void equalsSameArgsTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertTrue(event1.equals(event2));
  }

  @Test
  void equalsDifferentNumTicketsCapTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            30,
            0,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentTicketPriceInPenceTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            50,
            500,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentNumTicketsLeftTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    event2.setNumTicketsLeft(4);
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentTitleTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Empire Strikes Back",
            EventType.Movie,
            50,
            0,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentVenueAddressTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            50,
            0,
            "38.0594 122.6544",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentDescriptionTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            50,
            0,
            "51.998 0.741",
            "Viewing of 'The Empire Strikes Back' at Skywalker Ranch.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentStartDateTimeTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            50,
            0,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 8, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentEndDateTimeTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Movie,
            50,
            0,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 10, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentTypeTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
        new Event(
            42,
            "The Immitation Game",
            EventType.Theatre,
            50,
            0,
            "51.998 0.741",
            "Viewing of 'The Immitaiton Game' at Bletchley Park.",
            LocalDateTime.of(2023, 9, 30, 7, 30, 0),
            LocalDateTime.of(2023, 9, 30, 10, 30, 0),
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentTagsTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
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
            new EventTagCollection("Pets Allowed=No"));
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentStatusTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    event2.cancel();
    assertFalse(event1.equals(event2));
  }

  @Test
  void equalsDifferentReviewsTest() {
    Event event1 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    Event event2 =
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
            new EventTagCollection("Pets Allowed=No,Kids Allowed=Absolutely not"));
    event2.addReview(
        new Review(
            new Consumer("name", "email", "phone number", "address", "password"),
            event2,
            LocalDateTime.of(2023, 11, 30, 7, 30, 0),
            "This is a review."));
    assertFalse(event1.equals(event2));
  }
}
