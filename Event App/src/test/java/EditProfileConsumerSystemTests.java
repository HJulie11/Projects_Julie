import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import command.LoginCommand;
import command.LogoutCommand;
import command.RegisterConsumerCommand;
import command.UpdateConsumerProfileCommand;
import controller.Controller;
import model.EventTagCollection;
import org.junit.jupiter.api.Test;

public class EditProfileConsumerSystemTests extends ConsoleTest {

  @Test
  void editConsumerNotLoggedIn() {
    Controller controller = createController();
    startOutputCapture();
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare("USER_UPDATE_PROFILE_NOT_LOGGED_IN");
  }

  @Test
  void editConsumerAsStaff() {
    Controller controller = createController();
    startOutputCapture();
    createStaff(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            STAFF_PASSWORD,
            "Alice",
            STAFF_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            STAFF_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_STAFF_SUCCESS", "USER_LOGIN_SUCCESS", "CONSUMER_UPDATE_PROFILE_NOT_CONSUMER");
  }

  @Test
  void editConsumerWrongPassword() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD + "test",
            "Alice",
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS", "USER_UPDATE_PROFILE_WRONG_PASSWORD");
  }

  @Test
  void editConsumerName() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertTrue(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS", "CONSUMER_UPDATE_PROFILE_SUCCESS");
  }

  @Test
  void editConsumerNullName() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            null,
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL");
  }

  @Test
  void editConsumerNullOldPassword() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            null,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL");
  }

  @Test
  void editConsumerNullNewPassword() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            CONSUMER_ADDRESS,
            null,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL");
  }

  @Test
  void editConsumerNullEmail() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            null,
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL");
  }

  @Test
  void editConsumerNullPhone() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            null,
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL");
  }

  @Test
  void editConsumerTakenEmail() {
    Controller controller = createController();
    startOutputCapture();
    controller.runCommand(
        new RegisterConsumerCommand(
            "Domain Parker",
            "peter@parker.com",
            "01324456897",
            "55.94872684464941 -3.199892044473183", // Edinburgh Castle
            "park before it's popular!"));
    controller.runCommand(new LogoutCommand());
    createConsumer(controller);

    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            "peter@parker.com",
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "USER_LOGOUT_SUCCESS",
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "USER_UPDATE_PROFILE_EMAIL_ALREADY_IN_USE");
  }

  @Test
  void editConsumerNewEmailAndRelogin() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            "peter@parker.com",
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertTrue(updateCmd.getResult());

    controller.runCommand(new LogoutCommand());
    controller.runCommand(new LoginCommand("peter@parker.com", CONSUMER_PASSWORD));
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_SUCCESS",
        "USER_LOGOUT_SUCCESS",
        "USER_LOGIN_SUCCESS");
  }

  @Test
  void editConsumerEmailInvalidatesOldEmail() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            "peter@parker.com",
            "000",
            CONSUMER_ADDRESS,
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertTrue(updateCmd.getResult());

    controller.runCommand(new LogoutCommand());
    controller.runCommand(new LoginCommand(CONSUMER_EMAIL, CONSUMER_PASSWORD));
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_PROFILE_SUCCESS",
        "USER_LOGOUT_SUCCESS",
        "USER_LOGIN_EMAIL_NOT_REGISTERED");
  }

  @Test
  void editConsumerInvalidAddressFormat() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            "ADDRESS",
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "CONSUMER_UPDATE_INVALID_ADDRESS_FORMAT");
  }

  @Test
  void editConsumerAddressOutOfBounds() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    UpdateConsumerProfileCommand updateCmd =
        new UpdateConsumerProfileCommand(
            CONSUMER_PASSWORD,
            "Alice",
            CONSUMER_EMAIL,
            "000",
            "1.00000 1.000000",
            CONSUMER_PASSWORD,
            new EventTagCollection());
    controller.runCommand(updateCmd);
    assertFalse(updateCmd.getResult());
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS", "CONSUMER_UPDATE_ADDRESS_OUT_OF_BOUNDS");
  }

  // No way to add new tags without adding them to possible tags
}
