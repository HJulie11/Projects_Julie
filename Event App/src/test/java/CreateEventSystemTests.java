import static org.junit.jupiter.api.Assertions.assertNull;

import command.CreateEventCommand;
import command.LogoutCommand;
import command.RegisterStaffCommand;
import controller.Controller;
import java.time.LocalDateTime;
import model.Event;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class CreateEventSystemTests extends ConsoleTest {
  private static void registerPawsForAwwws(Controller controller) {
    controller.runCommand(
        new RegisterStaffCommand(
            "hasta@vista.baby", "very insecure password 123", "Nec temere nec timide"));
  }

  private static Event createEvent(
      Controller controller, LocalDateTime startDateTime, LocalDateTime endDateTime) {
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            100,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            startDateTime,
            endDateTime,
            new EventTagCollection());
    controller.runCommand(eventCmd);
    return eventCmd.getResult();
  }

  @Test
  void userNotStaff() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    createEvent(controller, 1, 1);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS", "CREATE_EVENT_USER_NOT_STAFF");
  }

  @Test
  void createNonTicketedEvent() {
    startOutputCapture();
    createStaffAndEvent(0, 1);
    stopOutputCaptureAndCompare(
        "REGISTER_STAFF_SUCCESS", "USER_LOGIN_SUCCESS", "CREATE_EVENT_SUCCESS");
  }

  @Test
  void createTicketedEvent() {
    startOutputCapture();
    Controller controller = createStaffAndEvent(1, 1);
    controller.runCommand(new LogoutCommand());
    stopOutputCaptureAndCompare(
        "REGISTER_STAFF_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CREATE_EVENT_SUCCESS",
        "USER_LOGOUT_SUCCESS");
  }

  @Test
  void createEventInThePast() {
    Controller controller = createController();
    startOutputCapture();
    registerPawsForAwwws(controller);
    Event event =
        createEvent(
            controller,
            LocalDateTime.now().minusDays(5),
            LocalDateTime.now().minusDays(5).plusHours(2));
    assertNull(event);
    stopOutputCaptureAndCompare(
        "REGISTER_STAFF_SUCCESS", "USER_LOGIN_SUCCESS", "CREATE_EVENT_IN_THE_PAST");
  }

  @Test
  void createEventWithEndBeforeStart() {
    Controller controller = createController();
    startOutputCapture();
    registerPawsForAwwws(controller);
    Event event =
        createEvent(
            controller,
            LocalDateTime.now().minusDays(5),
            LocalDateTime.now().minusDays(5).minusHours(2));
    assertNull(event);
    stopOutputCaptureAndCompare(
        "REGISTER_STAFF_SUCCESS", "USER_LOGIN_SUCCESS", "CREATE_EVENT_START_AFTER_END");
  }

  @Test
  void createEventsSameNameSameTimes() {
    Controller controller = createController();
    registerPawsForAwwws(controller);
    startOutputCapture();
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.of(2024, 06, 06, 12, 0),
            LocalDateTime.of(2024, 06, 06, 13, 0),
            new EventTagCollection()));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.of(2024, 06, 06, 12, 0),
            LocalDateTime.of(2024, 06, 06, 13, 0),
            new EventTagCollection()));
    stopOutputCaptureAndCompare("CREATE_EVENT_SUCCESS", "CREATE_EVENT_TITLE_AND_TIME_CLASH");
  }

  @Test
  void createNegativeTicketPrice() {
    Controller controller = createController();
    registerPawsForAwwws(controller);
    startOutputCapture();
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            -1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusHours(1),
            LocalDateTime.now().plusHours(2),
            new EventTagCollection()));
    stopOutputCaptureAndCompare("CREATE_EVENT_NEGATIVE_TICKET_PRICE");
  }

  @Test
  void createEventWithAddressNotInBounds() {
    Controller controller = createController();
    registerPawsForAwwws(controller);
    startOutputCapture();
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "1.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusHours(1),
            LocalDateTime.now().plusHours(2),
            new EventTagCollection()));
    stopOutputCaptureAndCompare("CREATE_EVENT_VENUE_NOT_IN_BOUNDARY");
  }

  @Test
  void createEventWithInvalidAddressFormat() {
    Controller controller = createController();
    registerPawsForAwwws(controller);
    startOutputCapture();
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "George Square Gardens, Edinburgh",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusHours(1),
            LocalDateTime.now().plusHours(2),
            new EventTagCollection()));
    stopOutputCaptureAndCompare("CREATE_EVENT_ADDRESS_NOT_IN_CORRECT_FORMAT");
  }

  // No function to add tag without it being added to possibletags
}
