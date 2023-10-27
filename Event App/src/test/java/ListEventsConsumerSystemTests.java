import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import command.AddEventTagCommand;
import command.CreateEventCommand;
import command.ListEventsCommand;
import command.LogoutCommand;
import command.UpdateConsumerProfileCommand;
import controller.Controller;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Set;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class ListEventsConsumerSystemTests extends ConsoleTest {

  @Test
  void validListEventsRequest() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new ListEventsCommand(true, false, LocalDate.now()));
    stopOutputCaptureAndCompare("LIST_EVENTS_SUCCESS");
  }

  @Test
  void validListEventsRequestNoEvents() {
    Controller controller = createController();
    createConsumer(controller);
    ListEventsCommand listEventsCmd = new ListEventsCommand(true, false, LocalDate.now());
    controller.runCommand(listEventsCmd);
    assertEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventUserNotLoggedInWithUserEventsOnly() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    controller.runCommand(new ListEventsCommand(true, false, LocalDate.now()));
    stopOutputCaptureAndCompare("LIST_EVENTS_NOT_LOGGED_IN");
  }

  @Test
  void listEventConsumerNoPreferenceOnTagNotDefault() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection("Pet friendly=Yes")));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertNotEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventConsumerNoPreferenceOnTagDefault() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection("Pet friendly=No")));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertNotEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventDefaultPreferenceNotTaggedInEvent() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection()));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    controller.runCommand(
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01324456897",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection("Pet friendly=No")));
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertNotEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventNonDefaultPreferenceNotTaggedInEvent() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection()));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    controller.runCommand(
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01324456897",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection("Pet friendly=Yes")));
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventConsumerDifferentPreferenceOnTag() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection("Pet friendly=Yes")));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    controller.runCommand(
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01324456897",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection("Pet friendly=No")));
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventConsumerCorrectPreferenceOnTag() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new AddEventTagCommand("Pet friendly", Set.of("Yes", "No"), "No"));
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            500,
            1,
            "55.94368888764689 -3.1888246174917114",
            "Come and enjoy some pets for pets",
            LocalDateTime.now().plusDays(1),
            LocalDateTime.now().plusDays(3),
            new EventTagCollection("Pet friendly=Yes")));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    controller.runCommand(
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01324456897",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection("Pet friendly=Yes")));
    ListEventsCommand listEventsCmd =
        new ListEventsCommand(true, false, LocalDate.now().plusDays(2));
    controller.runCommand(listEventsCmd);
    assertNotEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventUserNotLoggedInWithoutUserEventsOnly() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    startOutputCapture();
    controller.runCommand(new ListEventsCommand(false, false, LocalDate.now()));
    stopOutputCaptureAndCompare("LIST_EVENTS_SUCCESS");
  }

  @Test
  void listEventUserLoggedInWithUserEventsOnly() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new ListEventsCommand(true, false, LocalDate.now()));
    stopOutputCaptureAndCompare("LIST_EVENTS_SUCCESS");
  }
}
