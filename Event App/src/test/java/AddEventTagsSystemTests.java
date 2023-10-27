import static org.junit.jupiter.api.Assertions.assertEquals;

import command.AddEventTagCommand;
import controller.Controller;
import java.util.Set;
import model.EventTag;
import org.junit.jupiter.api.Test;

public class AddEventTagsSystemTests extends ConsoleTest {

  @Test
  void addValidTag() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    AddEventTagCommand addEventTagCmd =
        new AddEventTagCommand("Pet Friendly", Set.of("Yes", "No"), "No");
    controller.runCommand(addEventTagCmd);
    stopOutputCaptureAndCompare("ADD_TAG_SUCCESS");
    assertEquals(new EventTag(Set.of("Yes", "No"), "No"), addEventTagCmd.getResult());
  }

  @Test
  void consumerAddTag() {
    Controller controller = createController();
    createConsumer(controller);
    startOutputCapture();
    controller.runCommand(new AddEventTagCommand("Pet Friendly", Set.of("Yes", "No"), "No"));
    stopOutputCaptureAndCompare("ADD_TAG_USER_NOT_STAFF");
  }

  @Test
  void addExistingTagSameValues() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    createEventTag(controller);
    createEventTag(controller);
    stopOutputCaptureAndCompare("ADD_TAG_SUCCESS", "ADD_TAG_TAG_NAME_CLASH");
  }

  @Test
  void addExistingTagDifferentValues() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    createEventTag(controller);
    controller.runCommand(
        new AddEventTagCommand("Pet Friendly", Set.of("Maybe", "It depends"), "No"));
    stopOutputCaptureAndCompare("ADD_TAG_SUCCESS", "ADD_TAG_TAG_NAME_CLASH");
  }

  @Test
  void addTagOneValue() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    controller.runCommand(new AddEventTagCommand("Pet Friendly", Set.of("No"), "No"));
    stopOutputCaptureAndCompare("ADD_TAG_INSUFFICIENT_VALUES");
  }

  @Test
  void addTagNoValue() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    controller.runCommand(new AddEventTagCommand("Pet Friendly", Set.of(), ""));
    stopOutputCaptureAndCompare("ADD_TAG_INSUFFICIENT_VALUES");
  }

  @Test
  void addTagDefaultValueNotPossible() {
    Controller controller = createController();
    createStaff(controller);
    startOutputCapture();
    controller.runCommand(new AddEventTagCommand("Pet Friendly", Set.of("Yes", "Maybe"), "No"));
    stopOutputCaptureAndCompare("ADD_TAG_DEFAULT_VALUE_NOT_IN_POSSIBLE_VALUES");
  }
}
