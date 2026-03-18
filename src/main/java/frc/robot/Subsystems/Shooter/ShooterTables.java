package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTables {

	public static InterpolatingDoubleTreeMap ALLIANCE_TABLE = new InterpolatingDoubleTreeMap();

	static { // DISTANCE (M), ANGLE TODO: Converted to center of shooter to center of hub, distance is dubiously taken from CAD + field drawings. + 0.991 is what it used to be (measured in real)
		ALLIANCE_TABLE.put(6.0 + 1.0591546, 32.0); // Here so we have better bounds of table. Do not include if using regressions
		ALLIANCE_TABLE.put(5.7404 + 1.0591546, 30.0);
		ALLIANCE_TABLE.put(5.2959 + 1.0591546, 25.0);
		ALLIANCE_TABLE.put(4.7879 + 1.0591546, 20.0);
		ALLIANCE_TABLE.put(4.29895 + 1.0591546, 15.0);
		ALLIANCE_TABLE.put(3.3147 + 1.0591546, 10.0);
		ALLIANCE_TABLE.put(1.6764 + 1.0591546, 5.0);
		ALLIANCE_TABLE.put(0.3683 + 1.0591546, 0.0);
		ALLIANCE_TABLE.put(0.0 + 1.0591546, 0.0); // Here so we have better bounds of table. Do not include if using regressions
	}

	public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = ALLIANCE_TABLE;
	//public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = new InterpolatingDoubleTreeMap();

	// static {
	// 	SHUTTLING_TABLE.put(0.0, 0.0);
	// }
}
