import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import model.EventTagCollection;
import org.junit.jupiter.api.Test;

public class EventTagCollectionUnitTest extends ConsoleTest {
  @Test
  void gettingTagValueTest() {
    EventTagCollection tagCollection = new EventTagCollection("Hello=yes,hello=no");
    assertEquals(tagCollection.getValueFor("Hello"), "yes");
    assertEquals(tagCollection.getValueFor("hello"), "no");
  }

  @Test
  void equalsSameArgsTest() {
    EventTagCollection tagCollection = new EventTagCollection("Hello=yes,hello=no");
    EventTagCollection tagCollection2 = new EventTagCollection("Hello=yes,hello=no");
    assertTrue(tagCollection.equals(tagCollection2));
  }

  @Test
  void checkInitialisationTest() {
    EventTagCollection tagCollection = new EventTagCollection();
    assertTrue(tagCollection.getTags().isEmpty());
  }

  @Test
  void equalsSameObjectTest() {
    EventTagCollection tagCollection = new EventTagCollection("Hello=yes,hello=no");
    assertEquals(tagCollection, tagCollection);
  }

  @Test
  void toStringTest() {
    EventTagCollection tagCollection = new EventTagCollection("why=yes");
    assertEquals(tagCollection.toString(), "EventTagCollection{why=yes}");
  }

  @Test
  void overlappingTagTest() {
    EventTagCollection tagCollection = new EventTagCollection(("why=yes,why=no,why=maybe"));
    assertEquals(tagCollection.toString(), "EventTagCollection{why=maybe}");
  }
}
