package command;

import com.graphhopper.util.shapes.GHPoint;
import controller.Context;
import external.MapSystem;
import java.util.Map;
import model.Consumer;
import model.Event;
import model.EventTagCollection;
import model.User;
import view.IView;

/**
 * {@link UpdateConsumerProfileCommand} allows {@link Consumer}s to update their account details and
 * change the default Covid-19 preferences.
 */
public class UpdateConsumerProfileCommand extends UpdateProfileCommand {
  private final String oldPassword;
  private final String newName;
  private final String newEmail;
  private final String newPhoneNumber;
  private final String newAddress;
  private final String newPassword;
  private final EventTagCollection newPreferences;

  /**
   * @param oldPassword account password before the change, required for extra security
   *     verification. Must not be null
   * @param newName full name of the person holding this account. Must not be null
   * @param newEmail email address to use for this account. Must not be null
   * @param newPhoneNumber phone number to use for this account (used for notifying the {@link
   *     Consumer} of any {@link Event} cancellations that they have bookings for). Must not be null
   * @param newAddress updated Consumer address. Optional and may be null
   * @param newPassword password to use for this account. Must not be null
   * @param newPreferences a {@link EventTagCollection} object of Covid-19 preferences, used for
   *     filtering events in the {@link ListEventsCommand}. If null, default preferences are applied
   */
  public UpdateConsumerProfileCommand(
      String oldPassword,
      String newName,
      String newEmail,
      String newPhoneNumber,
      String newAddress,
      String newPassword,
      EventTagCollection newPreferences) {
    this.oldPassword = oldPassword;
    this.newName = newName;
    this.newEmail = newEmail;
    this.newPhoneNumber = newPhoneNumber;
    this.newAddress = newAddress;
    this.newPassword = newPassword;
    this.newPreferences = newPreferences == null ? new EventTagCollection() : newPreferences;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that oldPassword, newName, newEmail, newPhoneNumber, and newPassword are all not null
   * @verifies.that current user is logged in
   * @verifies.that oldPassword matches the current user's password
   * @verifies.that there is no other user already registered with the same email address as
   *     newEmail
   * @verifies.that currently logged-in user is a Consumer
   */
  @Override
  public void execute(Context context, IView view) {
    // Check all fields given
    if (oldPassword == null
        || newName == null
        || newEmail == null
        || newPhoneNumber == null
        || newPassword == null) {
      view.displayFailure(
          "UpdateConsumerProfileCommand",
          LogStatus.CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL,
          Map.of(
              "oldPassword",
              "***",
              "newName",
              String.valueOf(newName),
              "newEmail",
              String.valueOf(newEmail),
              "newPhoneNumber",
              String.valueOf(newPhoneNumber),
              "newPassword",
              "***",
              "newPreferences",
              newPreferences));
      successResult = false;
      return;
    }

    // Check address valid
    if (!(newAddress == "")) {
      MapSystem mapSystem = context.getMapSystem();
      if (!newAddress.matches(MapSystem.validCoordString)) {
        view.displayFailure(
            "RegisterConsumerCommand",
            UpdateConsumerProfileCommand.LogStatus.CONSUMER_UPDATE_INVALID_ADDRESS_FORMAT,
            Map.of("address", String.valueOf(newAddress)));
        successResult = false;
        return;
      }

      GHPoint point = mapSystem.convertToCoordinates(newAddress);
      if (!context.getMapSystem().isPointWithinMapBounds(point)) {
        view.displayFailure(
            "RegisterConsumerCommand",
            UpdateConsumerProfileCommand.LogStatus.CONSUMER_UPDATE_ADDRESS_OUT_OF_BOUNDS,
            Map.of("address", String.valueOf(newAddress)));
        successResult = false;
        return;
      }
    }

    // Check update valid
    if (isProfileUpdateInvalid(context, view, oldPassword, newEmail)) {
      successResult = false;
      return;
    }

    // Update user
    User currentUser = context.getUserState().getCurrentUser();
    if (!(currentUser instanceof Consumer)) {
      view.displayFailure(
          "UpdateConsumerProfileCommand", LogStatus.CONSUMER_UPDATE_PROFILE_NOT_CONSUMER);
      successResult = false;
      return;
    }

    changeUserEmail(context, newEmail);
    currentUser.updatePassword(newPassword);
    Consumer consumer = (Consumer) currentUser;
    consumer.setName(newName);
    consumer.setPhoneNumber(newPhoneNumber);
    consumer.setAddress(newAddress);
    consumer.setPreferences(newPreferences);

    view.displaySuccess(
        "UpdateConsumerProfileCommand",
        LogStatus.CONSUMER_UPDATE_PROFILE_SUCCESS,
        Map.of(
            "newName",
            newName,
            "newEmail",
            newEmail,
            "newPhoneNumber",
            newPhoneNumber,
            "newAddress",
            String.valueOf(newAddress),
            "newPassword",
            "***",
            "preferences",
            newPreferences));
    successResult = true;
  }

  private enum LogStatus {
    CONSUMER_UPDATE_PROFILE_FIELDS_CANNOT_BE_NULL,
    CONSUMER_UPDATE_PROFILE_NOT_CONSUMER,
    CONSUMER_UPDATE_PROFILE_SUCCESS,
    CONSUMER_UPDATE_INVALID_ADDRESS_FORMAT,
    CONSUMER_UPDATE_ADDRESS_OUT_OF_BOUNDS
  }
}
