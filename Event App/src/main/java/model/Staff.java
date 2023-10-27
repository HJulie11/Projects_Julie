package model;

/**
 * {@link Staff} is a user of the application, who represents an organisation that hosts {@link
 * Event}s. They can browse events, create new events, cancel events and see {@link Booking}s on
 * their own events.
 */
public class Staff extends User {
  /**
   * Create a new Staff member
   *
   * @param email email address of the organisation staff member (used as the account email address)
   * @param password password for this account
   */
  public Staff(String email, String password) {
    super(email, password);
  }

  @Override
  public String toString() {
    return "Staff";
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof Staff)) {
      return false;
    }
    Staff staff = (Staff) other;
    // Check same email
    if (!getEmail().equals(staff.getEmail())) {
      return false;
    }
    // Check same password hash
    if (!getPasswordHash().equals(staff.getPasswordHash())) {
      return false;
    }
    return true;
  }
}
