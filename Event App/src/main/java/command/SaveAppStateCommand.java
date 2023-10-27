package command;

import controller.Context;
import java.io.File;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;
import java.util.Map;
import model.Staff;
import model.User;
import view.IView;

/**
 * {@link SaveAppStateCommand} allows the logged in {@link Staff} member to save the system data to
 * a file.
 */
public class SaveAppStateCommand implements ICommand<Boolean> {
  private Boolean successResult;
  private String filename;

  /**
   * @param filename The path to the file to save the data to.
   */
  public SaveAppStateCommand(String filename) {
    if (!filename.endsWith(".dat")) {
      filename += ".dat";
    }
    this.filename = filename;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that the currently logged-in user is a Staff member
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user logged in and staff
    User currentUser = context.getUserState().getCurrentUser();
    if (currentUser == null) {
      view.displayFailure("SaveAppStateCommand", LogStatus.SAVE_APP_STATE_NOT_LOGGED_IN);
      successResult = false;
      return;
    }
    if (!(currentUser instanceof Staff)) {
      view.displayFailure("SaveAppStateCommand", LogStatus.SAVE_APP_STATE_LOGGED_IN_USER_NOT_STAFF);
      successResult = false;
      return;
    }

    // Save file
    File file = new File(filename);
    try (FileOutputStream fos = new FileOutputStream(file);
        ObjectOutputStream out = new ObjectOutputStream(fos)) {
      out.writeObject(context);
    } catch (Exception exception) {
      view.displayFailure(
          "SaveAppStateCommand",
          LogStatus.SAVE_APP_STATE_COULD_NOT_WRITE_TO_FILE,
          Map.of("exception", exception.getMessage()));
      successResult = false;
      file.delete();
      return;
    }

    successResult = true;
    view.displaySuccess("SaveAppStateCommand", successResult);
    return;
  }

  /**
   * @return true if successful and false otherwise
   */
  @Override
  public Boolean getResult() {
    return successResult;
  }

  private enum LogStatus {
    SAVE_APP_STATE_SUCCESS,
    SAVE_APP_STATE_COULD_NOT_WRITE_TO_FILE,
    SAVE_APP_STATE_NOT_LOGGED_IN,
    SAVE_APP_STATE_LOGGED_IN_USER_NOT_STAFF
  }
}
