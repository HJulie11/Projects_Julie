import static org.junit.jupiter.api.Assertions.*;

import command.GetEventDirectionsCommand;
import command.LogoutCommand;
import controller.Controller;
import model.Event;
import model.TransportMode;
import org.junit.jupiter.api.Test;

public class RequestDirectionsToVenueSystemTests extends ConsoleTest {

  @Test
  void validDirectionsRequest() {
    Controller controller = createController();
    createStaff(controller);
    Event event = createEvent(controller, 1, 1);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    GetEventDirectionsCommand getEventDirectionsCmd =
        new GetEventDirectionsCommand(event.getEventNumber(), TransportMode.car);
    controller.runCommand(getEventDirectionsCmd);
    stopOutputCaptureAndCompare("GET_EVENT_DIRECTIONS_SUCCESS");
    assertFalse(getEventDirectionsCmd.getResult().length == 0);
  }

  @Test
  void directionsEventNonExistent() {
    Controller controller = createController();
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new GetEventDirectionsCommand(1, TransportMode.car));
    stopOutputCaptureAndCompare("GET_EVENT_DIRECTIONS_NO_SUCH_EVENT");
  }

  @Test
  void directionsNoEventAddress() {
    Controller controller = createController();
    createStaff(controller);
    Event event = createEventNoAddress(controller, 1, 1);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new GetEventDirectionsCommand(event.getEventNumber(), TransportMode.car));
    stopOutputCaptureAndCompare("GET_EVENT_DIRECTIONS_NO_VENUE_ADDRESS");
  }

  @Test
  void directionsNotConsumer() {
    Controller controller = createController();
    createStaff(controller);
    Event event = createEvent(controller, 1, 1);
    startOutputCapture();
    controller.runCommand(new GetEventDirectionsCommand(event.getEventNumber(), TransportMode.car));
    stopOutputCaptureAndCompare("GET_EVENT_DIRECTIONS_USER_NOT_CONSUMER");
  }

  @Test
  void directionsNoUserAddress() {
    Controller controller = createController();
    createStaff(controller);
    Event event = createEvent(controller, 1, 1);
    controller.runCommand(new LogoutCommand());
    createConsumerNoAddress(controller);
    startOutputCapture();
    controller.runCommand(new GetEventDirectionsCommand(event.getEventNumber(), TransportMode.car));
    stopOutputCaptureAndCompare("GET_EVENT_DIRECTIONS_NO_CONSUMER_ADDRESS");
  }
}
