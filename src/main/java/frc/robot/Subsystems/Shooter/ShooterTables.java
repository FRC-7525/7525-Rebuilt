package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTables {

	public static InterpolatingDoubleTreeMap ALLIANCE_TABLE = new InterpolatingDoubleTreeMap();

	static { // DISTANCE (M), ANGLE TODO: THESE ARE CURRENTLY IN EDGE TO EDGE. MUST CONVERT TO SHOOTER TO HUB CENTER
		ALLIANCE_TABLE.put(6.0, 32.0); // Here so we have better bounds of table. Do not include if using regressions
		ALLIANCE_TABLE.put(5.7404, 30.0);
		ALLIANCE_TABLE.put(5.2959, 25.0);
		ALLIANCE_TABLE.put(4.7879, 20.0);
		ALLIANCE_TABLE.put(4.29895, 15.0);
		ALLIANCE_TABLE.put(3.3147, 10.0);
		ALLIANCE_TABLE.put(1.6764, 5.0);
		ALLIANCE_TABLE.put(0.3683, 0.0);
		ALLIANCE_TABLE.put(0.0, 0.0); // Here so we have better bounds of table. Do not include if using regressions
	}
	public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = ALLIANCE_TABLE; 
	//public static InterpolatingDoubleTreeMap SHUTTLING_TABLE = new InterpolatingDoubleTreeMap();

	// static {
	// 	SHUTTLING_TABLE.put(0.0, 0.0);
	// }
}
