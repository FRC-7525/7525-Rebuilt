package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTables {

	public static InterpolatingDoubleTreeMap ALLIANCE_TABLE = new InterpolatingDoubleTreeMap();

	// add 1.0591546 for edge to edge measurement
	static { // DISTANCE (M), ANGLE TODO: Converted to center of shooter to center of hub, distance is dubiously taken from CAD + field drawings. + 0.991 is what it used to be (measured in real)
		// ALLIANCE_TABLE.put(6.0 + 1.0591546, 32.0); // Here so we have better bounds of table. Do not include if using regressions

		ALLIANCE_TABLE.put(4.4958 + 1.0591546, 30.0);
		ALLIANCE_TABLE.put(4.064 + 1.0591546, 27.0);
		ALLIANCE_TABLE.put(3.7973 + 1.0591546, 25.0); //143.5 in 25 deg taped
		ALLIANCE_TABLE.put(3.4925 + 1.0591546, 23.0); //128 in 23 deg taped
		ALLIANCE_TABLE.put(3.4163 + 1.0591546, 21.0); //134.5 in 21 deg  mystery but works
		ALLIANCE_TABLE.put(2.4384 + 1.0591546, 18.0); // Taped shot
		ALLIANCE_TABLE.put(2.2479 + 1.0591546, 16.0); //88.5 in 16 deg taped
		ALLIANCE_TABLE.put(2.286 + 1.0591546, 13.0); //90 in 13 deg
		ALLIANCE_TABLE.put(1.13665 + 1.0591546, 7.0);
		ALLIANCE_TABLE.put(0.6985 + 1.0591546, 3.0);
		ALLIANCE_TABLE.put(0.35 + 1.0591546, 0.0); // Slightly fake
		ALLIANCE_TABLE.put(1.0591546, 0.0); // Nice end to table
	}

	public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = ALLIANCE_TABLE;
	//public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = new InterpolatingDoubleTreeMap();

	// static {
	// 	SHUTTLING_TABLE.put(0.0, 0.0);
	// }
}
