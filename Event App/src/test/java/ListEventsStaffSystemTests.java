import static org.junit.jupiter.api.Assertions.assertEquals;

import command.ListEventsCommand;
import command.LogoutCommand;
import controller.Controller;
import java.time.LocalDate;
import java.util.List;
import org.junit.jupiter.api.Test;

public class ListEventsStaffSystemTests extends ConsoleTest {

  @Test
  void validListEventReqeust() {
    Controller controller = createController();
    createStaff(controller);
    ListEventsCommand cmd = new ListEventsCommand(true, false, LocalDate.now());
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LIST_EVENTS_SUCCESS");
  }

  @Test
  void validListEventsRequestNoEvents() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new LogoutCommand());
    ListEventsCommand listEventsCmd = new ListEventsCommand(false, false, LocalDate.now());
    controller.runCommand(listEventsCmd);
    assertEquals(listEventsCmd.getResult(), List.of());
  }

  @Test
  void listEventStaffNotLoggedInWithUserEventsOnly() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new LogoutCommand());
    ListEventsCommand cmd = new ListEventsCommand(true, false, LocalDate.now());
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LIST_EVENTS_NOT_LOGGED_IN");
  }

  @Test
  void listEventStaffNotLoggedInWithoutUserEventsOnly() {
    Controller controller = createController();
    createStaff(controller);
    ListEventsCommand cmd = new ListEventsCommand(false, false, LocalDate.now());
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LIST_EVENTS_SUCCESS");
  }
}
