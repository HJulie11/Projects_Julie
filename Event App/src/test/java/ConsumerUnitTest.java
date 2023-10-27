import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.LocalDateTime;
import model.Booking;
import model.Consumer;
import model.EventTagCollection;
import model.Staff;
import org.junit.jupiter.api.Test;

public class ConsumerUnitTest extends ConsoleTest {
  @Test
  void addBookingTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Booking booking = new Booking(0, consumer, null, 5, LocalDateTime.of(2023, 1, 1, 0, 0, 0));
    consumer.addBooking(booking);
    assertEquals(1, consumer.getBookings().size());
    assertTrue(booking == consumer.getBookings().get(0));
  }

  @Test
  void addBookingMultipleTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Booking booking1 = new Booking(0, consumer, null, 5, LocalDateTime.of(2023, 1, 1, 0, 0, 0));
    Booking booking2 = new Booking(7, consumer, null, 10, LocalDateTime.of(2023, 2, 2, 0, 0, 0));
    consumer.addBooking(booking1);
    consumer.addBooking(booking2);
    assertEquals(2, consumer.getBookings().size());
    assertTrue(booking1 == consumer.getBookings().get(0));
    assertTrue(booking2 == consumer.getBookings().get(1));
  }

  @Test
  void notifyTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    startOutputCapture();
    consumer.notify("This is a message!");
    stopOutputCaptureAndCompare(
        "Message to alan.turing@bletchleypark.com and 01908 640404: This is a message!");
  }

  @Test
  void notifyNullTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    startOutputCapture();
    consumer.notify(null);
    stopOutputCaptureAndCompare("Message to alan.turing@bletchleypark.com and 01908 640404: null");
  }

  @Test
  void toStringTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    assertEquals(
        "Consumer{"
            + "bookings=[]"
            + ", name='Alan Turing'"
            + ", phoneNumber='01908 640404'"
            + ", address='51.998 0.741'"
            + ", preferences=EventTagCollection{}"
            + '}',
        consumer.toString());
  }

  @Test
  void toStringWithPreferencesTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    consumer.setPreferences(new EventTagCollection("tag1=val1,tag2=val2"));
    assertEquals(
        "Consumer{"
            + "bookings=[]"
            + ", name='Alan Turing'"
            + ", phoneNumber='01908 640404'"
            + ", address='51.998 0.741'"
            + ", preferences=EventTagCollection{tag1=val1,tag2=val2}"
            + '}',
        consumer.toString());
  }

  @Test
  void toStringWithBookingsTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    consumer.addBooking(new Booking(0, consumer, null, 5, LocalDateTime.of(2023, 1, 1, 0, 0, 0)));
    consumer.addBooking(new Booking(7, consumer, null, 10, LocalDateTime.of(2023, 2, 2, 0, 0, 0)));
    assertEquals(
        "Consumer{"
            + "bookings=["
            + "Booking{status=Active, bookingNumber=0, booker=Alan Turing, "
            + "event=null, numTickets=5, bookingDateTime=2023-01-01T00:00}, "
            + "Booking{status=Active, bookingNumber=7, booker=Alan Turing, "
            + "event=null, numTickets=10, bookingDateTime=2023-02-02T00:00}]"
            + ", name='Alan Turing'"
            + ", phoneNumber='01908 640404'"
            + ", address='51.998 0.741'"
            + ", preferences=EventTagCollection{}"
            + '}',
        consumer.toString());
  }

  @Test
  void equalsSameObjectTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    assertTrue(consumer.equals(consumer));
  }

  @Test
  void equalsSameArgsTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    // Should return false because password hashes will be different.
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsStaffTest() {
    Consumer consumer =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Staff staff = new Staff("alan.turing@bletchleypark.com", "VonNeumann123");
    assertFalse(consumer.equals(staff));
  }

  @Test
  void equalsDifferentEmailTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "ada.lovelace@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsDifferentNameTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Ada Lovelace",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsDifferentPhoneNumerTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "07872 644095",
            "51.998 0.741",
            "VonNeumann123");
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsDifferentAddressTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "0.741 51.998",
            "VonNeumann123");
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsDifferentPreferencesTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    consumer1.setPreferences(new EventTagCollection("tag=Yes"));
    consumer2.setPreferences(new EventTagCollection("tag=No"));
    assertFalse(consumer1.equals(consumer2));
  }

  @Test
  void equalsDifferentbookingsTest() {
    Consumer consumer1 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    Consumer consumer2 =
        new Consumer(
            "Alan Turing",
            "alan.turing@bletchleypark.com",
            "01908 640404",
            "51.998 0.741",
            "VonNeumann123");
    consumer1.addBooking(new Booking(0, consumer1, null, 1, LocalDateTime.now()));
    assertFalse(consumer1.equals(consumer2));
  }
}
