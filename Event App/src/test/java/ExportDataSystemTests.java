import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import command.SaveAppStateCommand;
import controller.Controller;
import java.io.File;
import org.junit.jupiter.api.Test;

public class ExportDataSystemTests extends ConsoleTest {
  private String filename = "Test";
  private String filenameWithExtension = filename + ".dat";

  @Test
  void saveAppStateWithoutFileExtension() {
    Controller controller = createController();
    createStaff(controller);
    SaveAppStateCommand cmd = new SaveAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("SAVE_APP_STATE_SUCCESS");
    assertTrue(cmd.getResult());
    File file = new File(filenameWithExtension);
    Boolean fileCreated = file.isFile();
    assertTrue(fileCreated);
    file.delete();
  }

  @Test
  void saveAppStateWithFileExtension() {
    Controller controller = createController();
    createStaff(controller);
    SaveAppStateCommand cmd = new SaveAppStateCommand(filenameWithExtension);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("SAVE_APP_STATE_SUCCESS");
    assertTrue(cmd.getResult());
    File file = new File(filenameWithExtension);
    Boolean fileCreated = file.isFile();
    assertTrue(fileCreated);
    file.delete();
  }

  @Test
  void saveAppStateWithoutUserLoggedIn() {
    Controller controller = createController();
    SaveAppStateCommand cmd = new SaveAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("SAVE_APP_STATE_NOT_LOGGED_IN");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    Boolean fileCreated = file.isFile();
    assertFalse(fileCreated);
    file.delete();
  }

  @Test
  void saveAppStateWithoutStaffLoggedIn() {
    Controller controller = createController();
    createConsumer(controller);
    SaveAppStateCommand cmd = new SaveAppStateCommand(filename);
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("SAVE_APP_STATE_LOGGED_IN_USER_NOT_STAFF");
    assertFalse(cmd.getResult());
    File file = new File(filenameWithExtension);
    Boolean fileCreated = file.isFile();
    assertFalse(fileCreated);
    file.delete();
  }

  @Test
  void saveAppStateToNonExistentFileLocation() {
    Controller controller = createController();
    createStaff(controller);
    SaveAppStateCommand cmd = new SaveAppStateCommand("/dev/null/test.dat");
    startOutputCapture();
    controller.runCommand(cmd);
    stopOutputCaptureAndCompare("SAVE_APP_STATE_COULD_NOT_WRITE_TO_FILE");
    assertFalse(cmd.getResult());
    File file = new File("/dev/null/test.dat");
    Boolean fileCreated = file.isFile();
    assertFalse(fileCreated);
    file.delete();
  }
}
