package frc.robot.Autonomous.ChoreoTrajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final double AvoidRadius = 1.318;
    public static final double AvoidX = 4.603;
    public static final double LowerAvoidY = 2.349;
    public static final double UpperAvoidY = 5.721;

    public static final class Poses {
        public static final Pose2d LeftAllianceTrenchStraighten = new Pose2d(3.87, 7.619, Rotation2d.fromRadians(3.142));
        public static final Pose2d LeftNeutralTrenchStraighten = new Pose2d(5.397, 7.619, Rotation2d.kZero);
        public static final Pose2d RightAllianceTrenchStraighten = new Pose2d(3.87, 0.451, Rotation2d.kZero);
        public static final Pose2d RightNeutralTrenchStraighten = new Pose2d(5.397, 0.451, Rotation2d.kZero);

        private Poses() {}
    }

    private ChoreoVars() {}
}