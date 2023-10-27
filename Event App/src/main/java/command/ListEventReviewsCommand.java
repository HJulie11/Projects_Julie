package command;

import controller.Context;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import model.Event;
import model.Review;
import view.IView;

/**
 * {@link ListEventReviewsCommand} allows any user to get a list of {@link Reviews} for any {@link
 * Event}s of the given title.
 */
public class ListEventReviewsCommand implements ICommand<List<Review>> {

  private final String eventTitle;
  private List<Review> reviewsResult;

  /**
   * @param eventTitle title of the event
   */
  public ListEventReviewsCommand(String eventTitle) {
    this.eventTitle = eventTitle;
  }

  /**
   * @param context object that provides access to global application state
   * @param view allows passing information to the user interface
   */
  @Override
  public void execute(Context context, IView view) {
    // Get list of events
    List<Event> eventsFound =
        context.getEventState().getAllEvents().stream()
            .filter(event -> event.getTitle().equals(eventTitle))
            .collect(Collectors.toList());

    // List reviews
    reviewsResult = new LinkedList<>();
    eventsFound.forEach((Event e) -> reviewsResult.addAll(e.getReviews()));
    view.displaySuccess(
        "ListEventReviewsCommand",
        LogStatus.LIST_REVIEWS_SUCCESS,
        Map.of("reviewListResult", reviewsResult));
  }

  /**
   * @return The list of {@link Review}s if successful and null otherwise
   */
  @Override
  public List<Review> getResult() {
    return reviewsResult;
  }

  private enum LogStatus {
    LIST_REVIEWS_SUCCESS
  }
}
