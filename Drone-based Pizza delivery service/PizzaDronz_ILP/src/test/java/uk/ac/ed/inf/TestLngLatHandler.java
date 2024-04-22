package uk.ac.ed.inf;

import org.junit.Rule;
import org.junit.Test;
import static org.junit.Assert.*;

import org.junit.rules.ExpectedException;
import uk.ac.ed.inf.ilp.data.*;

public class TestLngLatHandler {
    LngLatHandler lngLatHandling = new LngLatHandler();

    // Test valid distance calculation
    @Test
    public void testValidDistanceToCalculation() {
        double distance = lngLatHandling.distanceTo(new LngLat(0,0), new LngLat(3,4));
        assertEquals(5,distance,0);
    }

    // Test valid closeness check
    @Test
    public void testValidisCloseToCheck() {
        boolean close = lngLatHandling.isCloseTo(new LngLat(0,0), new LngLat(0,0.0001));
        assertTrue(close);
    }

    // Test if the function throws the exception
    @Rule
    public ExpectedException exception = ExpectedException.none();

//    @Test
//    public void testNotCloseToException(){
//        exception.expect(IllegalArgumentException.class);
//        exception.expectMessage("The given positions are not close to each other");
//        lngLatHandling.isCloseTo(new LngLat(0,0), new LngLat(0,1));
//    }
    // Test invalid closeness check
//    @Test
//    public void testInvalidClosenessCheck() {
//        boolean close = lngLatHandling.isCloseTo(new LngLat(0,0), new LngLat(0,0.1));
//        assertFalse(close);
//    }

    @Test
    public void testIsInRegion(){
        LngLat position = new LngLat(-3.188396, 55.944425);
        NamedRegion region = new NamedRegion("test", new LngLat[] {
                new LngLat(-3.192473,55.946233),
                new LngLat(-3.184319,55.946233),
                new LngLat(-3.192473,55.942617),
                new LngLat(-3.184319,55.942617)
        });
        assertTrue(lngLatHandling.isInRegion(position, region));
    }

    @Test
    public void testIsInRegionTrue() {
        NamedRegion region = new NamedRegion("test", new LngLat[]{new LngLat(0,0), new LngLat(0,1), new LngLat(1,1), new LngLat(1,0)});
        assertTrue(lngLatHandling.isInRegion(new LngLat(0.5, 0.5), region));
    }

    // Test invalid region check
    @Test
    public void testIsInRegionFalse() {
        NamedRegion region = new NamedRegion("test", new LngLat[]{new LngLat(0,0), new LngLat(0,1), new LngLat(1,1), new LngLat(1,0)});
        assertFalse(lngLatHandling.isInRegion(new LngLat(1.5, 1.5), region));
    }

    @Test
    public void testCorrectNextPositionCalculation() {
        LngLat newPosition = lngLatHandling.nextPosition(new LngLat(0, 0), 0);
        assertEquals(0.00015, newPosition.lng(),0);
        assertEquals(0, newPosition.lat(),0);
    }

}

