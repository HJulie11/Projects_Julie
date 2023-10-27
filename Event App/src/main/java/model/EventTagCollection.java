package model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class EventTagCollection implements Serializable {
  private Map<String, String> tags;

  /** Creates a new {@link EventTagCollection} */
  public EventTagCollection() {
    this.tags = new HashMap<>();
  }

  /**
   * Creates a new {@link EventTagCollection}
   *
   * @param tags A string of the form name1=value2,name2=value2
   */
  public EventTagCollection(String tags) {
    this.tags = new HashMap<String, String>();
    String[] keyValuePairs = tags.split(",");
    for (String pair : keyValuePairs) {
      String key = pair.split("=")[0];
      String value = pair.split("=")[1];
      this.tags.put(key, value);
    }
  }

  public String getValueFor(String tag) {
    return tags.get(tag);
  }

  public Map<String, String> getTags() {
    return tags;
  }

  @Override
  public String toString() {
    List<String> output = new ArrayList<String>();
    for (String key : tags.keySet()) {
      String tag = tags.get(key);
      output.add(key + "=" + tag);
    }
    return "EventTagCollection{" + String.join(",", output) + "}";
  }

  @Override
  public boolean equals(Object other) {
    // Check same class
    if (!(other instanceof EventTagCollection)) {
      return false;
    }
    EventTagCollection eventTagCollection = (EventTagCollection) other;
    // Check same tags
    if (this.tags.size() != eventTagCollection.tags.size()) {
      return false;
    }
    for (String key : this.tags.keySet()) {
      if (!eventTagCollection.tags.containsKey(key)) {
        return false;
      }
      if (!eventTagCollection.tags.get(key).equals(this.tags.get(key))) {
        return false;
      }
    }
    return true;
  }
}
