package model;

import java.io.Serializable;
import java.time.LocalDateTime;

public class Review implements Serializable {
  private final Consumer author;
  private final Event event;
  private final LocalDateTime creationDataTime;
  private final String content;

  public Review(Consumer author, Event event, LocalDateTime creationDataTime, String content) {
    this.author = author;
    this.event = event;
    this.creationDataTime = creationDataTime;
    this.content = content;
  }

  public Consumer getAuthor() {
    return author;
  }

  public Event getEvent() {
    return event;
  }

  public LocalDateTime getCreationDataTime() {
    return creationDataTime;
  }

  public String getContent() {
    return content;
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof Review)) {
      return false;
    }
    Review review = (Review) other;
    // Check same content
    if (!content.equals(review.content)) {
      return false;
    }
    // Check same time
    if (!creationDataTime.equals(review.creationDataTime)) {
      return false;
    }
    // Check same author
    if (!author.equals(review.author)) {
      return false;
    }
    // Check same title
    if (!event.getTitle().equals(review.event.getTitle())) {
      return false;
    }
    return false;
  }
}
