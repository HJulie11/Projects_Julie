import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import command.AddEventTagCommand;
import command.BookEventCommand;
import command.CreateEventCommand;
import command.ListEventsCommand;
import command.RegisterConsumerCommand;
import command.RegisterStaffCommand;
import controller.Context;
import controller.Controller;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import model.Booking;
import model.Event;
import model.EventTag;
import model.EventTagCollection;
import model.EventType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

public class ConsoleTest {
  private static PrintStream downstream;
  private final Pattern commandPattern =
      Pattern.compile("(?<callerName>[A-Za-z]+)(?<additionalInfo>.*?) => (?<result>[A-Z0-9_]+)");
  private ByteArrayOutputStream out;

  @BeforeAll
  static void saveDownstream() {
    downstream = System.out;
  }

  @BeforeEach
  void printTestName(TestInfo testInfo) {
    System.out.println(testInfo.getDisplayName());
  }

  protected void startOutputCapture() {
    // NOTE: be careful, if the captured output exceeds 8192 bytes, the remainder will be lost!
    out = new ByteArrayOutputStream(8192);
    System.setOut(new PrintStream(out));
  }

  protected void stopOutputCaptureAndCompare(String... expected) {
    ByteArrayInputStream in = new ByteArrayInputStream(out.toByteArray());
    BufferedReader br = new BufferedReader(new InputStreamReader(in));

    try {
      String line;
      int idx;
      for (line = br.readLine(), idx = 0;
          line != null && idx < expected.length;
          line = br.readLine(), ++idx) {
        downstream.println(line);
        Matcher m = commandPattern.matcher(line);
        if (m.find()) {
          assertEquals(expected[idx], m.group("result"));
        }
        // otherwise output includes a line that is not in command format
        // this happens when consumers are notified about cancellations
        // which is safe to ignore
      }

      assertEquals(expected.length, idx);
      assertNull(line);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @AfterEach
  void restoreDownstream() {
    System.setOut(downstream);
    System.out.println("---");
  }

  protected static Controller createController() {
    return new Controller(
        new Context(
            "The University of Edinburgh",
            "55.94747223411703 -3.187300017491497", // Old College, South Bridge, Edinburgh
            "epay@ed.ac.uk",
            "Nec temere nec timide"),
        new TestView());
  }

  protected static final String STAFF_EMAIL = "bring-in-the-cash@pawsforawwws.org";
  protected static final String STAFF_PASSWORD = "very insecure password 123";

  protected static void createStaff(Controller controller) {
    controller.runCommand(
        new RegisterStaffCommand(
            "bring-in-the-cash@pawsforawwws.org",
            "very insecure password 123",
            "Nec temere nec timide"));
  }

  protected static Event createEvent(Controller controller, int numTickets, int eventDelayHours) {
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            numTickets,
            0,
            "55.94368888764689 -3.1888246174917114", // George Square Gardens, Edinburgh
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusHours(eventDelayHours),
            LocalDateTime.now().plusHours(eventDelayHours + 1),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    return eventCmd.getResult();
  }

  protected static void createListOfEvents(Controller controller) {
    String[] addresses = {
      "55.8627889 -4.2510763", // Glasgow queen street
      "55.94368888764689 -3.1888246174917114", // Edinburgh castle
      "57.1601958 -2.0917136", // Pittodrie stadium, Aberdeen
      "57.4790124 -4.225739", // Inverness town centre
      "55.9902293 -3.3966999", // South queensferry
      "58.6432136 -3.0837196"
    }; // john o'groats
    for (int i = 0; i < 6; i++) {
      CreateEventCommand eventCmd =
          new CreateEventCommand(
              String.valueOf(i),
              EventType.Theatre,
              1,
              0,
              addresses[i],
              "Please be prepared to pay 2.50 pounds on entry",
              LocalDateTime.now().plusHours(1),
              LocalDateTime.now().plusHours(2),
              new EventTagCollection());
      controller.runCommand(eventCmd);
    }
  }

  protected static Event createEventNoAddress(
      Controller controller, int numTickets, int eventDelayHours) {
    CreateEventCommand eventCmd =
        new CreateEventCommand(
            "Puppies against depression",
            EventType.Theatre,
            numTickets,
            0,
            "",
            "Please be prepared to pay 2.50 pounds on entry",
            LocalDateTime.now().plusHours(eventDelayHours),
            LocalDateTime.now().plusHours(eventDelayHours + 1),
            new EventTagCollection());
    controller.runCommand(eventCmd);
    return eventCmd.getResult();
  }

  protected static Controller createStaffAndEvent(int numTickets, int eventDelayHours) {
    Controller controller = createController();
    createStaff(controller);
    createEvent(controller, numTickets, eventDelayHours);
    return controller;
  }

  protected static final String CONSUMER_EMAIL = "i-would-never-steal-a@dog.xd";
  protected static final String CONSUMER_PASSWORD = "123456";
  protected static final String CONSUMER_ADDRESS = "55.94872684464941 -3.199892044473183";

  protected static void createConsumer(Controller controller) {
    controller.runCommand(
        new RegisterConsumerCommand(
            "Chihuahua Fan",
            CONSUMER_EMAIL,
            "01324456897",
            "55.94872684464941 -3.199892044473183", // Edinburgh Castle
            CONSUMER_PASSWORD));
  }

  protected static void createConsumerNoAddress(Controller controller) {
    controller.runCommand(
        new RegisterConsumerCommand(
            "Chihuahua Fan", CONSUMER_EMAIL, "01324456897", "", CONSUMER_PASSWORD));
  }

  protected static Booking createConsumerAndBookFirstEvent(
      Controller controller, int numTicketsRequested) {
    createConsumer(controller);
    ListEventsCommand eventsCmd = new ListEventsCommand(false, false, null);
    controller.runCommand(eventsCmd);
    List<Event> events = eventsCmd.getResult();
    long firstEventNumber = events.get(0).getEventNumber();
    BookEventCommand bookCmd = new BookEventCommand(firstEventNumber, numTicketsRequested);
    controller.runCommand(bookCmd);
    return bookCmd.getResult();
  }

  protected static EventTag createEventTag(Controller controller) {
    AddEventTagCommand addEventTagCmd =
        new AddEventTagCommand("Pet Friendly", Set.of("Yes", "No"), "No");
    controller.runCommand(addEventTagCmd);
    return addEventTagCmd.getResult();
  }

  protected static List<Event> getAllEvents(Controller controller) {
    ListEventsCommand eventsCmd = new ListEventsCommand(false, false, null);
    controller.runCommand(eventsCmd);
    return eventsCmd.getResult();
  }

  protected static List<Event> getUserEvents(Controller controller) {
    ListEventsCommand eventsCmd = new ListEventsCommand(true, false, null);
    controller.runCommand(eventsCmd);
    return eventsCmd.getResult();
  }
}
