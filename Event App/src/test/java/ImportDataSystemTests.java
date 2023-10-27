import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import command.AddEventTagCommand;
import command.BookEventCommand;
import command.CreateEventCommand;
import command.LoadAppStateCommand;
import command.LoginCommand;
import command.LogoutCommand;
import command.RegisterConsumerCommand;
import command.SaveAppStateCommand;
import controller.Controller;
import java.io.File;
import java.time.LocalDateTime;
import java.util.HashSet;
import java.util.Set;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.Test;

public class ImportDataSystemTests extends ConsoleTest {
  private String filename = "Test";
  private String filenameWithExtension = filename + ".dat";

  @Test
  void loadAppStateWithoutFileExtension() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_SUCCESS");
    assertTrue(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithFileExtension() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    LoadAppStateCommand cmd = new LoadAppStateCommand(filenameWithExtension);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_SUCCESS");
    assertTrue(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithoutUserLoggedIn() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_NOT_LOGGED_IN");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithoutStaffLoggedIn() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    createConsumer(controller);
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_LOGGED_IN_USER_NOT_STAFF");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateFromNonExistentFileLocation() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand("/dev/null/test.dat"));
    LoadAppStateCommand cmd = new LoadAppStateCommand("/dev/null/test.dat");
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_COULD_NOT_READ_FILE");
    assertFalse(cmd.getResult());
    File file = new File("/dev/null/test.dat");
    file.delete();
  }

  @Test
  void loadAppStateWithTagsMismatch() {
    Controller controller = createController();
    createStaff(controller);
    createEventTag(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    createStaff(controller);
    Set<String> tagValues = new HashSet<>();
    for (int i = 0; i < 3; i++) {
      tagValues.add("value" + i); // Different to original
    }
    controller.runCommand(new AddEventTagCommand("Pet Friendly", tagValues, "value1"));
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_TAGS_MISMATCH");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithUsersMismatch() {
    Controller controller = createController();
    createConsumer(controller);
    controller.runCommand(new LogoutCommand());
    createStaff(controller);
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    controller.runCommand(
        new RegisterConsumerCommand(
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01279644095", // Different to original
            "55.94872684464941 -3.199892044473183",
            CONSUMER_PASSWORD));
    controller.runCommand(new LogoutCommand());
    createStaff(controller);
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_USERS_MISMATCH");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithEventsMismatch() {
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            10,
            0,
            "55.94368888764689 -3.1888246174917114",
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.of(2024, 1, 1, 1, 1, 1),
            LocalDateTime.of(2024, 1, 1, 5, 1, 1),
            new EventTagCollection()));
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    createStaff(controller);
    controller.runCommand(
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Movie, // Different to original
            10,
            0,
            "55.94368888764689 -3.1888246174917114",
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.of(2024, 1, 1, 1, 1, 1),
            LocalDateTime.of(2024, 1, 1, 5, 1, 1),
            new EventTagCollection()));
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_EVENTS_MISMATCH");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  @Test
  void loadAppStateWithBookingsMismatch() {
    CreateEventCommand createEventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Movie,
            10,
            0,
            "55.94368888764689 -3.1888246174917114",
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.of(2024, 1, 1, 1, 1, 1),
            LocalDateTime.of(2024, 1, 1, 5, 1, 1),
            new EventTagCollection());
    RegisterConsumerCommand registerConsumerCmd =
        new RegisterConsumerCommand(
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01279644095",
            "55.94872684464941 -3.199892044473183",
            CONSUMER_PASSWORD);
    Controller controller = createController();
    createStaff(controller);
    controller.runCommand(createEventCmd);
    controller.runCommand(new LogoutCommand());
    controller.runCommand(registerConsumerCmd);
    controller.runCommand(new BookEventCommand(1, 1));
    controller.runCommand(new LogoutCommand());
    controller.runCommand(
        new LoginCommand("bring-in-the-cash@pawsforawwws.org", "very insecure password 123"));
    controller.runCommand(new SaveAppStateCommand(filename));
    controller = createController();
    createStaff(controller);
    controller.runCommand(createEventCmd);
    controller.runCommand(new LogoutCommand());
    controller.runCommand(registerConsumerCmd);
    controller.runCommand(new BookEventCommand(1, 1));
    controller.runCommand(new LogoutCommand());
    controller.runCommand(
        new LoginCommand("bring-in-the-cash@pawsforawwws.org", "very insecure password 123"));
    LoadAppStateCommand cmd = new LoadAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("LOAD_APP_STATE_USERS_MISMATCH");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    file.delete();
  }

  // ? loads appends all data
  // @Test
  // void loadAppStateAppendsAllSavedData() {
  // Controller controller = createStaffAndEvent(0, 0);
  // createEventTag(controller);
  // createConsumer(controller);
  // controller.runCommand(new LogoutCommand());

  // }
}
