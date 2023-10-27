import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import command.LogoutCommand;
import command.RegisterConsumerCommand;
import controller.Controller;
import org.junit.jupiter.api.Test;

public class RegisterConsumerSystemTests extends ConsoleTest {

  @Test
  void userAlreadyLoggedIn() {
    Controller controller = createController();
    startOutputCapture();
    createConsumer(controller);
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            null,
            "john@hotmail.com",
            "01937563",
            "55.94872684464941 -3.199892044473183",
            "password");
    controller.runCommand(regCommand);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS", "USER_REGISTER_LOGGED_IN");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegisterNullName() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            null,
            "john@hotmail.com",
            "01937563",
            "55.94872684464941 -3.199892044473183",
            "password");
    controller.runCommand(regCommand);
    assertNull(regCommand.getResult());
    stopOutputCaptureAndCompare("USER_REGISTER_FIELDS_CANNOT_BE_NULL");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegisterNullEmail() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "john", null, "01937563", "55.94872684464941 -3.199892044473183", "password");
    controller.runCommand(regCommand);
    assertNull(regCommand.getResult());
    stopOutputCaptureAndCompare("USER_REGISTER_FIELDS_CANNOT_BE_NULL");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegisterNullPhone() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "john", "john@hotmail.com", null, "55.94872684464941 -3.199892044473183", "password");
    controller.runCommand(regCommand);
    assertNull(regCommand.getResult());
    stopOutputCaptureAndCompare("USER_REGISTER_FIELDS_CANNOT_BE_NULL");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegisterNullPassword() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "john", "john@hotmail.com", "01937563", "55.94872684464941 -3.199892044473183", null);
    controller.runCommand(regCommand);
    assertNull(regCommand.getResult());
    stopOutputCaptureAndCompare("USER_REGISTER_FIELDS_CANNOT_BE_NULL");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegisterEmailAlreadyExists() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "John",
            "john@hotmail.com",
            "01937563",
            "55.94872684464941 -3.199892044473183",
            "password");
    controller.runCommand(regCommand);
    controller.runCommand(new LogoutCommand());
    RegisterConsumerCommand newRegCommand =
        new RegisterConsumerCommand(
            "Jonny",
            "john@hotmail.com",
            "01937563",
            "55.94872684464941 -3.199892044473183",
            "password");
    controller.runCommand(newRegCommand);
    stopOutputCaptureAndCompare(
        "REGISTER_CONSUMER_SUCCESS",
        "USER_LOGIN_SUCCESS",
        "USER_LOGOUT_SUCCESS",
        "USER_REGISTER_EMAIL_ALREADY_REGISTERED");
    assertNull(newRegCommand.getResult());
  }

  @Test
  void userRegistersInvalidAddressFormat() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand("John", "john@hotmail.com", "012", " 012801", "password");
    controller.runCommand(regCommand);
    stopOutputCaptureAndCompare("USER_REGISTER_INVALID_ADDRESS_FORMAT");
    assertNull(regCommand.getResult());
  }

  @Test
  void userRegistersWithAddressOutOfBounds() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "John", "John@hotmail.com", "0192834", "44.590467 98.437500", "password");
    controller.runCommand(regCommand);
    stopOutputCaptureAndCompare("USER_ADDRESS_OUT_OF_BOUNDS");
    assertNull(regCommand.getResult());
  }

  @Test
  void userSuccessfullyRegister() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand(
            "John",
            "john@hotmail.com",
            "01937563",
            "55.94872684464941 -3.199892044473183",
            "password");
    controller.runCommand(regCommand);
    stopOutputCaptureAndCompare("REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS");
    assertNotNull(regCommand.getResult());
  }

  @Test
  void userSuccessfullyRegisterNoAddress() {
    Controller controller = createController();
    startOutputCapture();
    RegisterConsumerCommand regCommand =
        new RegisterConsumerCommand("John", "john@hotmail.com", "01937563", "", "password");
    controller.runCommand(regCommand);
    stopOutputCaptureAndCompare("REGISTER_CONSUMER_SUCCESS", "USER_LOGIN_SUCCESS");
    assertNotNull(regCommand.getResult());
  }
}
