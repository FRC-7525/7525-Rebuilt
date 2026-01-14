package frc.robot.subsystems.AA;

import java.util.List;
import static frc.robot.subsystems.AA.RepulsorFieldPlanner.PointObstacle;


import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.AA.RepulsorFieldPlanner.GuidedObstacle;
import frc.robot.subsystems.AA.RepulsorFieldPlanner.HorizontalObstacle;
import frc.robot.subsystems.AA.RepulsorFieldPlanner.Obstacle;
import frc.robot.subsystems.AA.RepulsorFieldPlanner.VerticalObstacle;


/*
 * EVERYTHING IS IN METERS AND METRIC UNITS
 */
public final class RepulsorFieldPlannerConstants {
    public static final double GOAL_STRENGTH = 0.65; //changable if you want pull force towards goal to be stornger/weaker
	public static final double FIELD_LENGTH = 16.42;
	public static final double FIELD_WIDTH = 8.16;	
	public static final int ARROWS_ON_X_AXIS = 40; // change if you want to modiify the number of arrows on the x-axis
	public static final int ARROWS_ON_Y_AXIS = 20; // likewise for y axis. more arrows = better viz but slower performance

    public static final class DefaultObstalces {
        public static final List<Obstacle> FIELD_OBSTACLES = List.of(
			new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6, 2.625)),
			new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6, 4)),
			new PointObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6,5.375) )
			);

	    public static final List<Obstacle> WALLS = List.of(
			new HorizontalObstacle(0.5, true, 0, Meters.of(1)),
			new HorizontalObstacle(0.5, true, FIELD_WIDTH, Meters.of(1)),
			new VerticalObstacle(0.5, true, 0, Meters.of(1)),
			new VerticalObstacle(0.5, true, FIELD_LENGTH, Meters.of(1)),
			new VerticalObstacle(0.5, true, FIELD_LENGTH, Meters.of(FIELD_LENGTH))
	    );
    }
}