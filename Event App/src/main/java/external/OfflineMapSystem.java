package external;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.CHProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.util.Translation;
import com.graphhopper.util.shapes.GHPoint;
import java.util.Locale;
import model.TransportMode;

/**
 * An implementation of {@link MapSystem}. The Offline map system uses GraphHopper's offline library
 * and OpenStreetMap map data for Scotland. Geocoding (lookup of place names and converting them
 * into coordinates) is not supported, so the whole system works with coordinates only.
 */
public class OfflineMapSystem implements MapSystem {
  private final GraphHopper hopper;

  public OfflineMapSystem() {
    this.hopper = new GraphHopper();
    hopper.setOSMFile("scotland-latest.osm.pbf");
    hopper.setGraphHopperLocation("routing-graph-cache");
    hopper.setProfiles(
        new Profile("car").setVehicle("car").setWeighting("fastest").setTurnCosts(false),
        new Profile("bike").setVehicle("bike").setWeighting("fastest").setTurnCosts(false),
        new Profile("wheelchair")
            .setVehicle("wheelchair")
            .setWeighting("fastest")
            .setTurnCosts(false),
        new Profile("foot").setVehicle("foot").setWeighting("fastest").setTurnCosts(false));
    hopper.getCHPreparationHandler().setCHProfiles(new CHProfile("car"));
    hopper.importOrLoad();
  }

  @Override
  public GHPoint convertToCoordinates(String address) {
    return GHPoint.fromString(String.join(",", address.split(" ")));
  }

  @Override
  public boolean isPointWithinMapBounds(GHPoint point) {
    return hopper.getBaseGraph().getBounds().contains(point.getLat(), point.getLon());
  }

  @Override
  public ResponsePath routeBetweenPoints(TransportMode mode, GHPoint start, GHPoint destination) {
    GHRequest req =
        new GHRequest(start, destination).setProfile(mode.toString()).setLocale(Locale.UK);
    GHResponse rsp = hopper.route(req);
    if (rsp.hasErrors()) {
      throw new RuntimeException(rsp.getErrors().toString());
    }
    return rsp.getBest();
  }

  @Override
  public Translation getTranslation() {
    return hopper.getTranslationMap().getWithFallBack(Locale.UK);
  }

  public void close() {
    hopper.close();
  }
}
