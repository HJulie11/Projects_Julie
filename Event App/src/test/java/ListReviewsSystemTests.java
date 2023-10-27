import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import command.CreateEventCommand;
import command.ListEventReviewsCommand;
import command.LogoutCommand;
import command.ReviewEventCommand;
import controller.Controller;
import java.time.LocalDateTime;
import java.util.List;
import java.util.concurrent.TimeUnit;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class ListReviewsSystemTests extends ConsoleTest {

  @Test
  void listReviewSuccess() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Opera",
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
    Thread.sleep(TimeUnit.SECONDS.toMillis(6)); // Waits for event to finish
    ReviewEventCommand reviewEventCommand = new ReviewEventCommand(1, "Great");
    controller.runCommand(reviewEventCommand);
    startOutputCapture();
    ListEventReviewsCommand listReviewsCmd = new ListEventReviewsCommand("Opera");
    controller.runCommand(listReviewsCmd);
    stopOutputCaptureAndCompare("LIST_REVIEWS_SUCCESS");
    assertNotEquals(listReviewsCmd.getResult(), List.of());
  }

  @Test
  void listReviewsNoReviews() {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Opera",
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
    ListEventReviewsCommand listReviewsCmd = new ListEventReviewsCommand("Opera");
    controller.runCommand(listReviewsCmd);
    System.out.println(listReviewsCmd.getResult());
    System.out.println(List.of());
    assertEquals(listReviewsCmd.getResult(), List.of());
  }

  @Test
  void listReviewsWrongTitle() throws InterruptedException {
    Controller controller = createController();
    createStaff(controller);
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Opera",
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
    Thread.sleep(TimeUnit.SECONDS.toMillis(6)); // Waits for event to finish
    ReviewEventCommand reviewEventCommand = new ReviewEventCommand(1, "Great");
    controller.runCommand(reviewEventCommand);
    ListEventReviewsCommand listReviewsCmd = new ListEventReviewsCommand("Theatre");
    controller.runCommand(listReviewsCmd);
    System.out.println(listReviewsCmd.getResult());
    System.out.println(List.of());
    assertEquals(listReviewsCmd.getResult(), List.of());
  }
}
