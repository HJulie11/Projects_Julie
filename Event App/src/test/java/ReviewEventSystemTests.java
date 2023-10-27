import static org.junit.jupiter.api.Assertions.assertNull;

import command.CreateEventCommand;
import command.LogoutCommand;
import command.ReviewEventCommand;
import controller.Controller;
import java.time.LocalDateTime;
import java.util.concurrent.TimeUnit;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class ReviewEventSystemTests extends ConsoleTest {

  @Test
  void reviewEventSuccess() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            10,
            0,
            "55.94368888764689 -3.1888246174917114", // George Square Gardens, Edinburgh
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusSeconds(1),
            LocalDateTime.now().plusSeconds(3),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    controller.runCommand(new LogoutCommand());
    createConsumerAndBookFirstEvent(controller, 1);
    Thread.sleep(TimeUnit.SECONDS.toMillis(6));
    startOutputCapture();
    ReviewEventCommand reviewEventCommand = new ReviewEventCommand(1, "Great");
    controller.runCommand(reviewEventCommand);
    stopOutputCaptureAndCompare("REVIEW_EVENT_SUCCESS");
  }

  @Test
  void reviewEventNotFound() {
    Controller controller = createController();
    createConsumer(controller);
    startOutputCapture();
    ReviewEventCommand reviewCommand = new ReviewEventCommand(123, "Great");
    assertNull(reviewCommand.getResult());
    controller.runCommand(reviewCommand);
    stopOutputCaptureAndCompare("REVIEW_EVENT_NOT_FOUND");
  }

  @Test
  void reviewEventNotOver() {
    Controller controller = createStaffAndEvent(4, 7);
    controller.runCommand(new LogoutCommand());
    createConsumerAndBookFirstEvent(controller, 1);
    startOutputCapture();
    ReviewEventCommand reviewCommand = new ReviewEventCommand(1, "great");
    controller.runCommand(reviewCommand);
    assertNull(reviewCommand.getResult());
    stopOutputCaptureAndCompare("REVIEW_EVENT_NOT_OVER");
  }

  @Test
  void reviewEventNotConsumer() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            10,
            0,
            "55.94368888764689 -3.1888246174917114", // George Square Gardens, Edinburgh
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusSeconds(1),
            LocalDateTime.now().plusSeconds(3),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    Thread.sleep(TimeUnit.SECONDS.toMillis(6));
    startOutputCapture();
    ReviewEventCommand reviewCommand = new ReviewEventCommand(1, "Great");
    controller.runCommand(reviewCommand);
    assertNull(reviewCommand.getResult());
    stopOutputCaptureAndCompare("REVIEW_EVENT_USER_NOT_CONSUMER");
  }

  @Test
  void reviewEventNoValidBookings() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            10,
            0,
            "55.94368888764689 -3.1888246174917114", // George Square Gardens, Edinburgh
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusSeconds(1),
            LocalDateTime.now().plusSeconds(3),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);
    Thread.sleep(TimeUnit.SECONDS.toMillis(6));
    startOutputCapture();
    ReviewEventCommand reviewCommand = new ReviewEventCommand(1, "Good");
    controller.runCommand(reviewCommand);
    stopOutputCaptureAndCompare("REVIEW_EVENT_NO_VALID_BOOKING");
  }
}
