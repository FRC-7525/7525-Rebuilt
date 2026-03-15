package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTables {
    public static InterpolatingDoubleTreeMap ALLIANCE_TABLE = new InterpolatingDoubleTreeMap();
    static { // DISTANCE, ANGLE
        ALLIANCE_TABLE.put(0.0, 0.0);
    }
    public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = new InterpolatingDoubleTreeMap();
    static {
        SHUTTLING_TABLE.put(0.0, 0.0);
    }
}
