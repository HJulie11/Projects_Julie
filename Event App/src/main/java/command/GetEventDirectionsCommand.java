package command;

import com.graphhopper.ResponsePath;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.Translation;
import com.graphhopper.util.shapes.GHPoint;
import controller.Context;
import external.MapSystem;
import java.util.Map;
import model.Consumer;
import model.Event;
import model.TransportMode;
import model.User;
import view.IView;

/**
 * {@link GetEventDirectionsCommand} allows a logged in {@link Consumer} to request directions to
 * given {@link Event}.
 */
public class GetEventDirectionsCommand implements ICommand<String[]> {

  private String[] directionsResult;
  private final long eventNumber;
  private final TransportMode transportMode;

  /**
   * @param eventNumber number of the event
   * @param transportMode method of transportation
   */
  public GetEventDirectionsCommand(long eventNumber, TransportMode transportMode) {
    this.directionsResult = new String[1000];
    this.eventNumber = eventNumber;
    this.transportMode = transportMode;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   * @verifies.that there is an event corresponding to the provided eventNumber
   * @verifies.that the event includes a venueAddress
   * @verifies.that the current user is a {@link Consumer}
   * @verifies.that the {@link Consumer}'s profile includes an address
   */
  @Override
  public void execute(Context context, IView view) {
    // Check event exists
    Event event = context.getEventState().findEventByNumber(eventNumber);
    if (event == null) {
      view.displayFailure(
          "GetEventDirectionsCommand",
          LogStatus.GET_EVENT_DIRECTIONS_NO_SUCH_EVENT,
          Map.of("eventNumber", eventNumber));
      directionsResult = null;
      return;
    }

    // Check event has address
    if (event.getVenueAddress() == null || event.getVenueAddress().isBlank()) {
      view.displayFailure(
          "GetEventDirectionsCommand",
          LogStatus.GET_EVENT_DIRECTIONS_NO_VENUE_ADDRESS,
          Map.of("event", event));
      directionsResult = null;
      return;
    }

    // Check user logged in and consuemr
    User user = context.getUserState().getCurrentUser();
    if (!(user instanceof Consumer)) {
      view.displayFailure(
          "GetEventDirectionsCommand",
          LogStatus.GET_EVENT_DIRECTIONS_USER_NOT_CONSUMER,
          Map.of("user", String.valueOf(user)));
      directionsResult = null;
      return;
    }

    // Check consumer has address
    if (((Consumer) user).getAddress() == null || ((Consumer) user).getAddress().isBlank()) {
      view.displayFailure(
          "GetEventDirectionsCommand",
          LogStatus.GET_EVENT_DIRECTIONS_NO_CONSUMER_ADDRESS,
          Map.of("user", user));
      directionsResult = null;
      return;
    }

    // Get directions
    MapSystem mapSystem = context.getMapSystem();
    GHPoint userAddress = mapSystem.convertToCoordinates(((Consumer) user).getAddress());
    GHPoint venueAddress = mapSystem.convertToCoordinates(event.getVenueAddress());
    ResponsePath path = mapSystem.routeBetweenPoints(transportMode, userAddress, venueAddress);
    InstructionList il = path.getInstructions();
    Translation tr = mapSystem.getTranslation();
    directionsResult[0] = "Total Distance: " + path.getDistance();
    for (int i = 1; i < il.size(); i++) {
      directionsResult[i] =
          "distance "
              + il.get(i).getDistance()
              + "for instruction: "
              + il.get(i).getTurnDescription(tr);
    }

    view.displaySuccess(
        "getEventDirectionsCommands",
        LogStatus.GET_EVENT_DIRECTIONS_SUCCESS,
        Map.of("directionsResult", directionsResult));
  }

  /**
   * @return The array of directions if successful and null otherwise
   * @note the first results element indicates total path distance. The remaining results elements
   *     give step-by-step directions with distance and instructions for each step.
   */
  @Override
  public String[] getResult() {
    return directionsResult;
  }

  private enum LogStatus {
    GET_EVENT_DIRECTIONS_SUCCESS,
    GET_EVENT_DIRECTIONS_NO_SUCH_EVENT,
    GET_EVENT_DIRECTIONS_NO_VENUE_ADDRESS,
    GET_EVENT_DIRECTIONS_USER_NOT_CONSUMER,
    GET_EVENT_DIRECTIONS_NO_CONSUMER_ADDRESS
  }
}
