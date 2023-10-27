package model;

import java.util.LinkedList;
import java.util.List;

/**
 * {@link Consumer} represents a user of the application, who can browse {@link Event}s and book
 * {@link Event}s.
 */
public class Consumer extends User {
  private final List<Booking> bookings;
  private String name;
  private String phoneNumber;
  private String address;
  private EventTagCollection preferences;

  /**
   * Create a new Consumer with an empty list of bookings and default Covid-19 preferences
   *
   * @param name full name of the Consumer
   * @param email email address of the Consumer (used to log in to the application and for event
   *     cancellation notifications)
   * @param phoneNumber phone number of the Consumer (used for event cancellation notifications)
   * @param address address of the Consumer (optional)
   * @param password password used to log in to the application
   */
  public Consumer(String name, String email, String phoneNumber, String address, String password) {
    super(email, password);
    this.name = name;
    this.phoneNumber = phoneNumber;
    this.address = address;
    this.bookings = new LinkedList<>();
    this.preferences = new EventTagCollection();
  }

  public void addBooking(Booking booking) {
    bookings.add(booking);
  }

  public String getName() {
    return name;
  }

  public void setName(String newName) {
    this.name = newName;
  }

  public EventTagCollection getPreferences() {
    return preferences;
  }

  public void setPreferences(EventTagCollection preferences) {
    this.preferences = preferences;
  }

  public String getAddress() {
    return address;
  }

  public void setAddress(String address) {
    this.address = address;
  }

  public List<Booking> getBookings() {
    return bookings;
  }

  /**
   * Mock method: print out a message to STDOUT. A real implementation would send an email and/or
   * text to the {@link Consumer}'s {@link #phoneNumber}.
   *
   * @param message message from an {@link Staff} regarding an event cancellation
   */
  public void notify(String message) {
    System.out.println("Message to " + getEmail() + " and " + phoneNumber + ": " + message);
  }

  public String getPhoneNumber() {
    return phoneNumber;
  }

  public void setPhoneNumber(String newPhoneNumber) {
    this.phoneNumber = newPhoneNumber;
  }

  @Override
  public String toString() {
    return "Consumer{"
        + "bookings="
        + bookings
        + ", name='"
        + name
        + '\''
        + ", phoneNumber='"
        + phoneNumber
        + '\''
        + ", address='"
        + address
        + '\''
        + ", preferences="
        + preferences
        + '}';
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof Consumer)) {
      return false;
    }
    Consumer consumer = (Consumer) other;
    // Check same email
    if (!getEmail().equals(consumer.getEmail())) {
      return false;
    }
    // Check same password hash
    if (!getPasswordHash().equals(consumer.getPasswordHash())) {
      return false;
    }
    // Check same name
    if (!name.equals(consumer.name)) {
      return false;
    }
    // Check same phone number
    if (!phoneNumber.equals(consumer.phoneNumber)) {
      return false;
    }
    // Check same address
    if (!address.equals(consumer.address)) {
      return false;
    }
    // Check same preferences
    if (!preferences.equals(consumer.preferences)) {
      return false;
    }
    // Check same bookings
    if (bookings.size() != consumer.bookings.size()) {
      return false;
    }
    for (int i = 0; i < bookings.size(); i++) {
      if (!bookings.get(i).equals(consumer.bookings.get(i))) {
        return false;
      }
    }
    return true;
  }
}
