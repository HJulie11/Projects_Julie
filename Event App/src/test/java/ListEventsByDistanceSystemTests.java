import static org.junit.jupiter.api.Assertions.assertEquals;

import command.ListEventsMaxDistanceCommand;
import command.LogoutCommand;
import controller.Controller;
import java.time.LocalDate;
import java.util.List;
import model.TransportMode;
import org.junit.jupiter.api.Test;

public class ListEventsByDistanceSystemTests extends ConsoleTest {

  @Test
  void validListDistance() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(
        new ListEventsMaxDistanceCommand(
            true, true, LocalDate.of(2020, 1, 8), TransportMode.car, 1000000));
    stopOutputCaptureAndCompare("LIST_EVENTS_MAX_DISTANCE_SUCCESS");
  }

  @Test
  void listDistanceZeroMaxDistance() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    ListEventsMaxDistanceCommand listEventsMaxDistanceCmd =
        new ListEventsMaxDistanceCommand(
            true, true, LocalDate.of(2020, 1, 8), TransportMode.car, 0);
    controller.runCommand(listEventsMaxDistanceCmd);
    stopOutputCaptureAndCompare("LIST_EVENTS_MAX_DISTANCE_SUCCESS");
    assertEquals(listEventsMaxDistanceCmd.getResult(), List.of());
  }

  @Test
  void userNotConsumer() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    startOutputCapture();
    controller.runCommand(
        new ListEventsMaxDistanceCommand(
            true, true, LocalDate.of(2020, 1, 8), TransportMode.car, 1000000));
    stopOutputCaptureAndCompare("LIST_EVENTS_MAX_DISTANCE_USER_NOT_CONSUMER");
  }

  @Test
  void userAddressNotValid() {
    Controller controller = createController();
    createStaff(controller);
    createListOfEvents(controller);
    controller.runCommand(new LogoutCommand());
    createConsumerNoAddress(controller);
    startOutputCapture();
    controller.runCommand(
        new ListEventsMaxDistanceCommand(
            true, true, LocalDate.of(2020, 1, 8), TransportMode.car, 1000000));
    stopOutputCaptureAndCompare("LIST_EVENTS_MAX_DISTANCE_USER_HAS_NO_SETUP_ADDRESS");
  }
}
