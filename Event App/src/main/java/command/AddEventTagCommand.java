package command;

import controller.Context;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import model.EventTag;
import model.Staff;
import model.User;
import view.IView;

/**
 * {@link AddEventTagCommand} allows a logged in {@link Staff} member to create a new {@link
 * EventTag}.
 */
public class AddEventTagCommand implements ICommand<EventTag> {
  private final String tagName;
  private final Set<String> tagValues;
  private final String defaultValue;
  private EventTag eventTagResult;

  /**
   * @param tagName name of the tag
   * @param tagValues possible values of the tag
   * @param defaultValue default value of the tag
   */
  public AddEventTagCommand(String tagName, Set<String> tagValues, String defaultValue) {
    this.tagName = tagName;
    this.tagValues = tagValues;
    this.defaultValue = defaultValue;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that the current user is a {@link Staff} member
   * @verifies.that the new tag name doesn't clash with any existing tags
   * @verifies.that there are at least 2 tag values
   * @verifies.that the default tag value is in the list of possible tag values
   */
  @Override
  public void execute(Context context, IView view) {
    // Check user is logged in and staff
    User currentUser = context.getUserState().getCurrentUser();
    if (!(currentUser instanceof Staff)) {
      view.displayFailure(
          "AddEventTagCommand",
          LogStatus.ADD_TAG_USER_NOT_STAFF,
          Map.of("currentUser", currentUser != null ? currentUser : "none"));
      return;
    }

    // Check tag doesn't already exist
    HashMap<String, EventTag> possibleTags = context.getEventState().getPossibleTags();
    if (possibleTags.containsKey(tagName)) {
      view.displayFailure(
          "AddEventTagCommand", LogStatus.ADD_TAG_TAG_NAME_CLASH, Map.of("tagName", tagName));
      return;
    }

    // Check tag has at least 2 values
    if (tagValues.size() < 2) {
      view.displayFailure(
          "AddEventCommand", LogStatus.ADD_TAG_INSUFFICIENT_VALUES, Map.of("tagValues", tagValues));
      return;
    }

    // Check tag's default value is one of the possible values
    if (!(tagValues.contains(defaultValue))) {
      view.displayFailure(
          "AddEventCommand",
          LogStatus.ADD_TAG_DEFAULT_VALUE_NOT_IN_POSSIBLE_VALUES,
          Map.of("defaultValue", defaultValue));
      return;
    }

    // Create new tag
    EventTag eventTag = context.getEventState().createEventTag(tagName, tagValues, defaultValue);
    view.displaySuccess(
        "AddEventCommand",
        LogStatus.ADD_TAG_SUCCESS,
        Map.of("tagName", tagName, "tagValues", tagValues, "defaultValue", defaultValue));
    eventTagResult = eventTag;
  }

  /**
   * @return The {@link EventTag} instance if successful and null otherwise
   */
  @Override
  public EventTag getResult() {
    return eventTagResult;
  }

  private enum LogStatus {
    ADD_TAG_SUCCESS,
    ADD_TAG_USER_NOT_STAFF,
    ADD_TAG_TAG_NAME_CLASH,
    ADD_TAG_INSUFFICIENT_VALUES,
    ADD_TAG_DEFAULT_VALUE_NOT_IN_POSSIBLE_VALUES,
  }
}
