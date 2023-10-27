package model;

import java.io.Serializable;
import java.util.Set;

public class EventTag implements Serializable {
  public Set<String> values;
  public String defaultValue;

  public EventTag(Set<String> values, String defaultValue) {
    this.values = values;
    this.defaultValue = defaultValue;
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof EventTag)) {
      return false;
    }
    EventTag eventTag = (EventTag) other;
    // Check same default value
    if (!(defaultValue.equals(eventTag.defaultValue))) {
      return false;
    }
    // Check same values
    if (values.size() != eventTag.values.size()) {
      return false;
    }
    for (String value : values) {
      if (!eventTag.values.contains(value)) {
        return false;
      }
    }
    return true;
  }
}
