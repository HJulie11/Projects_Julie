import command.BookEventCommand;
import command.CreateEventCommand;
import command.LogoutCommand;
import controller.Controller;
import java.time.LocalDateTime;
import java.util.concurrent.TimeUnit;
import model.Event;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class BookEventSystemTests extends ConsoleTest {

  @Test
  void bookTicketedEvent() {
    Controller controller = createStaffAndEvent(1, 1);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    createConsumerAndBookFirstEvent(controller, 1);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "LIST_EVENTS_SUCCESS",
        "BOOK_EVENT_SUCCESS");
  }

  @Test
  void bookTicketedEventMultiBookings() {
    Controller controller = createStaffAndEvent(5, 1);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 3));
    controller.runCommand(new BookEventCommand(1, 2));
    stopOutputCaptureAndCompare("BOOK_EVENT_SUCCESS", "BOOK_EVENT_SUCCESS");
  }

  @Test
  void overbookTicketedEvent() {
    Controller controller = createStaffAndEvent(1, 1);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    createConsumerAndBookFirstEvent(controller, 2);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "LIST_EVENTS_SUCCESS",
        "BOOK_EVENT_NOT_ENOUGH_TICKETS_LEFT");
  }

  @Test
  void requestZeroTickets() {
    Controller controller = createStaffAndEvent(1, 1);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    createConsumerAndBookFirstEvent(controller, 0);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "LIST_EVENTS_SUCCESS",
        "BOOK_EVENT_INVALID_NUM_TICKETS");
  }

  @Test
  void overbookTicketedEventMultiBookings() {
    Controller controller = createStaffAndEvent(2, 1);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 1));
    controller.runCommand(new BookEventCommand(1, 2));
    stopOutputCaptureAndCompare("BOOK_EVENT_SUCCESS", "BOOK_EVENT_NOT_ENOUGH_TICKETS_LEFT");
  }

  @Test
  void bookEventNotLoggedIn() {
    Controller controller = createStaffAndEvent(5, 1);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 5));
    stopOutputCaptureAndCompare("BOOK_EVENT_USER_NOT_CONSUMER");
  }

  @Test
  void bookEventStaff() {
    Controller controller = createStaffAndEvent(5, 1);
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 5));
    stopOutputCaptureAndCompare("BOOK_EVENT_USER_NOT_CONSUMER");
  }

  @Test
  void bookNonExistingEvent() {
    Controller controller = createStaffAndEvent(5, 1);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1324, 1));
    stopOutputCaptureAndCompare("BOOK_EVENT_EVENT_NOT_FOUND");
  }

  @Test
  void bookInactiveEvent() {
    Controller controller = createController();
    createStaff(controller);
    Event event = createEvent(controller, 1, 1);
    event.cancel();
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 1));
    stopOutputCaptureAndCompare("BOOK_EVENT_EVENT_NOT_ACTIVE");
  }

  @Test
  void bookFinishedEvent() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            1,
            0,
            "55.94368888764689 -3.1888246174917114", // George Square Gardens, Edinburgh
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusSeconds(1),
            LocalDateTime.now().plusSeconds(3),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    Thread.sleep(TimeUnit.SECONDS.toMillis(6)); // Waits for event to finish
    startOutputCapture();
    controller.runCommand(new BookEventCommand(1, 1));
    stopOutputCaptureAndCompare("BOOK_EVENT_ALREADY_OVER");
  }

  // No check for failed payment since it will always be true
}
