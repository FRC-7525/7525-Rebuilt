package frc.robot.subsystems.AA;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.AA.RepulsorFieldPlannerConstants.*;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.AA.RepulsorFieldPlannerConstants.DefaultObstalces;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

// Stolen from Team 6995 in LPS

public class RepulsorFieldPlanner {
    /**
     * Basis of all other obstacles
     */
    public abstract static class Obstacle {
        double strength;
        boolean shouldRepel;

        /**
         * 
         * @param currentPosition Current robot position on the field
         * @param goalPosition Robot target position
         * @return Force acting on the robot from the current obstacle.
         */
        public abstract Force getForceAtPosition(Translation2d currentPosition, Translation2d goalPosition);

		/**
		 * 
		 * @param strength Strength repulsion force of the obstacle
		 * @param shouldRepel Determines whether obstacle should repel or attract.
		 */
        public Obstacle(double strength, boolean shouldRepel) {
            this.strength = strength;
            this.shouldRepel = shouldRepel;
        }

        /**
         * Calculates the force magnitude based on the distance from the obstacle.
		 * 
         * @param distance Distance between the robot and the obstacle in meters.
         * @return magnitude of the force acting on the robot from the obstacle.
         */
        protected double calculateForceMagnitude(double distance) {
            double forceMag = strength / (0.00001 + Math.abs(distance * distance));
			forceMag *= shouldRepel ? 1 : -1;
			return forceMag;
        }

        /**
         * 
         * @param distance Distance between the robot and the obstacle in meters.
         * @param falloff Falloff distance in meter. Once past the falloff distance, the force will be 0.
         * @return magnitude of the force acting on the robot from the obstacle.
         */
        protected double calculateForceMagnitude(double distance, double falloff) {
			var original = strength / (0.00001 + Math.abs(distance * distance));
			var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
			return Math.max(original - falloffMag, 0) * (shouldRepel ? 1 : -1);
		}
    }

    /**
     * Obstacle with an associated position.
	 * <p> Should be used for objects that should be avoided
     */
    public static class PointObstacle extends Obstacle {
        Translation2d obstacleLocation;
        double obstacleRadius;

		/**
		 * Creates a it bstacle object
		 * @param strength Strength repulsion force of the obstacle
		 * @param shouldRepel Determines whether obstacle should repel or attract.
		 * @param obstacleRadius Size of the obstacle
		 * @param obstacleLocation Position of the obstacle
		 */
        public PointObstacle(double strength, boolean shouldRepel, Distance obstacleRadius, Translation2d obstacleLocation) {
            super(strength, shouldRepel);
            this.obstacleLocation = obstacleLocation;
            this.obstacleRadius = obstacleRadius.in(Meters);
        }

		/**
		 * @param currentPosition Current robot position on the field
		 * @param targetPosition The target position of the robot
		 */
		public Force getForceAtPosition(Translation2d currentPosition, Translation2d targetPosition) {
			double distance = obstacleLocation.getDistance(currentPosition);
			if (distance > 4) { // if further than 4 meters, obstacle does not apply force
				return new Force();
			}

			// First calculates outward force
			double outwardForceMag = calculateForceMagnitude(obstacleLocation.getDistance(currentPosition) - obstacleRadius);
			Force initalForce = new Force(outwardForceMag, currentPosition.minus(obstacleLocation).getAngle());

			// Determines whether to go ccw or cw around the object
			Rotation2d theta = targetPosition.minus(currentPosition).getAngle().minus(currentPosition.minus(obstacleLocation).getAngle());
			double mag = (outwardForceMag * Math.signum(Math.sin(theta.getRadians() / 2))) / 2; // fancy trick to determine whether to go ccw or cw around the object.

			return initalForce
				.rotateBy(Rotation2d.kCCW_90deg) // rotates so the object does not go directly backward/forward into the reef
				.div(initalForce.getNorm())
				.times(mag)
				.plus(initalForce);
		}
    }

	/**
	 * Obstacle that provides a stronger guiding force around a object.
	 * This should be used for large objects like the reef in 2025
	 */
    public static class GuidedObstacle extends Obstacle {
        Translation2d obstacleLocation;
        double obstacleRadius;

        public GuidedObstacle(double strength, boolean shouldRepel, Distance obstacleRadius, Translation2d obstacleLocation) {
            super(strength, shouldRepel);
            this.obstacleLocation = obstacleLocation;
            this.obstacleRadius = obstacleRadius.in(Meters);
        }

        /**
         * 
		 * <p> Essentially the same force calculation as PointObstacle, but with an extra "setpoint" 
         * @param currentPosition Position of the robot on the field
		 * @param targetPosition The target position of the robot
		 * @return Guiding force that pushes the robot towards the target.
         */
        public Force getForceAtPosition(Translation2d currentPosition, Translation2d targetPosition) {

			//normal repulsion force calculation
            double initialMag = calculateForceMagnitude(obstacleLocation.getDistance(currentPosition));
			Force initialForce = new Force(initialMag, currentPosition.minus(obstacleLocation).getAngle());

			// Additionally "setpoint" force calculation to help guide hte object further.
			Translation2d targetToObstacle = obstacleLocation.minus(targetPosition);
			Rotation2d targetToObstacleAngle = targetToObstacle.getAngle();
            Translation2d sidewaysCircle = new Translation2d(obstacleRadius, targetToObstacle.getAngle()).plus(obstacleLocation);
            double sidewaysMag = calculateForceMagnitude(sidewaysCircle.getDistance(currentPosition)); 

			// Determines whether the robot should go ccw or cw around the obstacle
            Rotation2d sidewaysTheta = targetPosition
                .minus(currentPosition)
                .getAngle()
                .minus(currentPosition.minus(sidewaysCircle).getAngle());
            sidewaysMag *= Math.signum(Math.sin(sidewaysTheta.getRadians()));
			// Adds the inital force witht he additional guiding force together
            Rotation2d sidewaysAngle = targetToObstacleAngle.rotateBy(Rotation2d.kCCW_90deg);

            return new Force(sidewaysMag, sidewaysAngle).plus(initialForce);
        }
    }

    /**
     * Horizontal Wall
	 * 
     */
    public static class HorizontalObstacle extends Obstacle {
		double y;
        double falloff;

		/**
		 * Creates a horizontal obstacle object
		 * @param strength Strength repulsion force of the obstacle
		 * @param shouldRepel Determines whether obstacle should repel or attract.
		 * @param yPosition Y position of the obstacle
		 * @param falloffRadius Falloff radius of the obstacle
		 * <p> The falloff radius is the distance from the obstacle where the force will be 0.
		 */
		public HorizontalObstacle(double strength, boolean shouldRepel, double yPosition, Distance falloffRadius) {
			super(strength, shouldRepel);
			this.y = yPosition;
            this.falloff = falloffRadius.in(Meters);
		}

		/**
		 * <p> Standard force calculation, but only considers the Y component
		 * 
		 * @param position Current position of the robot on the field
		 * @param target Target position of the robot
		 * @return Returns force acting on the robot. There is not a X component of the force.
		 */
		public Force getForceAtPosition(Translation2d position, Translation2d target) {
			return new Force(0, calculateForceMagnitude(y - position.getY(), falloff));
		}
	}

	/**
	 * Vertical Wall
	 */
    public static class VerticalObstacle extends Obstacle {
		double x;
        double falloff;

		/**
		 * Creates a vertical obstacle object
		 * @param strength Strength repulsion force of the obstacle
		 * @param shouldRepel Determines whether obstacle should repel or attract.
		 * @param xPosition X position of the obstacle
		 * @param falloffRadius Falloff radius of the obstacle
		 * <p> The falloff radius is the distance from the obstacle where the force will be 0.
		 */
		public VerticalObstacle(double strength, boolean shouldRepel, double xPosition, Distance falloffRadius) {
			super(strength, shouldRepel);
			this.x = xPosition;
            this.falloff = falloffRadius.in(Meters);
		}

		/**
		 * <p> Standard force calculation, but only considers the X component
		 * 
		 * @param position Current position of the robot on the field
		 * @param target Target position of the robot
		 * @return Returns force acting on the robot. There is not a Y component of the force.
		 */
		public Force getForceAtPosition(Translation2d position, Translation2d target) {
			return new Force(calculateForceMagnitude(x - position.getX(), falloff), 0);
		}
	}

    private Optional<Translation2d> goalOpt = Optional.empty();

	private List<Obstacle> allFieldObstacles = new ArrayList<>();
	private List<Obstacle> fieldObstacles = new ArrayList<>();
	private List<Obstacle> wallObstacles = new ArrayList<>();

	private static final int ARROWS_X = ARROWS_ON_X_AXIS;
	private static final int ARROWS_Y = ARROWS_ON_Y_AXIS;
	private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

	private boolean simulateArrows;
	private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

    private boolean useGoalInArrows = false;
	private boolean useObstaclesInArrows = true;
	private boolean useWallsInArrows = true;
    private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

	/**
	 * Creates a RepulsorFieldPlanner object with the given obstacles and walls.
	 * <p> If the obstacles or walls are empty, it will use the default obstacles and walls.
	 * @param fieldObstacles List of obstacles in the field. If empty, will use default obstacles.
	 * <p> The default obstacles are defined in {@link DefaultObstalces#FIELD_OBSTACLES}.
	 * @param walls. List of walls in the field. If empty, will use default walls.
	 * <p> The default walls are defined in {@link DefaultObstalces#WALLS}.
	 * @param simulateArrows Determines whether arrows should be simulated or not.
	 * <p> ARROWS SHOULD ONLY BE SIMULATED IN SIMULATION MODE
	*/
public RepulsorFieldPlanner(List<Obstacle> fieldObstacles, List<Obstacle> walls, boolean simulateArrows) { //TODO change this constructor since it's goofy
		this.fieldObstacles = fieldObstacles.isEmpty() ? DefaultObstalces.FIELD_OBSTACLES : fieldObstacles;
		this.wallObstacles = walls.isEmpty() ? DefaultObstalces.WALLS : walls;
		allFieldObstacles.addAll(DefaultObstalces.FIELD_OBSTACLES);
		allFieldObstacles.addAll(DefaultObstalces.WALLS);
		this.simulateArrows = simulateArrows;

		for (int i = 0; i < ARROWS_SIZE; i++) {
			arrows.add(new Pose2d());
		}

		// Creates a bunch of configurables in NT for repulsor. Most of it is just visualization
		{
			var topic = NetworkTableInstance.getDefault().getBooleanTopic("useGoalInArrows");
			topic.publish().set(useGoalInArrows);
			NetworkTableListener.createListener(topic, EnumSet.of(Kind.kValueAll), event -> {
				useGoalInArrows = event.valueData.value.getBoolean();
				if (simulateArrows) updateArrows();
			});
			topic.subscribe(useGoalInArrows);
		}

		{
			var topic = NetworkTableInstance.getDefault().getBooleanTopic("useObstaclesInArrows");
			topic.publish().set(useObstaclesInArrows);
			NetworkTableListener.createListener(topic, EnumSet.of(Kind.kValueAll), event -> {
				useObstaclesInArrows = event.valueData.value.getBoolean();
				if (simulateArrows) updateArrows();
			});
			topic.subscribe(useObstaclesInArrows);
		}

		{
			var topic = NetworkTableInstance.getDefault().getBooleanTopic("useWallsInArrows");
			topic.publish().set(useWallsInArrows);
			NetworkTableListener.createListener(topic, EnumSet.of(Kind.kValueAll), event -> {
				useWallsInArrows = event.valueData.value.getBoolean();
				if (simulateArrows) updateArrows();
			});
			topic.subscribe(useWallsInArrows);
		}

		NetworkTableInstance.getDefault()
			.startEntryDataLog(
				DataLogManager.getLog(),
				"SmartDashboard/Alerts",
				"SmartDashboard/Alerts"
			);
	}

	/**
	 * Updates arrow display
	 */
    void updateArrows() {
		if (!simulateArrows) return;

		for (int x = 0; x <= ARROWS_X; x++) {
			for (int y = 0; y <= ARROWS_Y; y++) {
				var translation = new Translation2d(
					(x * FIELD_LENGTH) / ARROWS_X,
					(y * FIELD_WIDTH) / ARROWS_Y
				);
				var force = Force.kZero;
				if (useObstaclesInArrows) force = force.plus(
					getObstacleForce(translation, goal().getTranslation())
				);
				if (useWallsInArrows) force = force.plus(
					getWallForce(translation, goal().getTranslation())
				);
				if (useGoalInArrows) {
					force = force.plus(getGoalForce(translation, goal().getTranslation()));
				}
				if (force.getNorm() < 1e-6) {
					arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
				} else {
					var rotation = force.getAngle();

					arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(translation, rotation));
				}
			}
		}
	}

	/**
	 * Obtains the goal pose of the robot.
	 * <p> If the goal is not set, it will return a zero pose.
	 * @return Current goal pose or zero pose if goal is not set.
	 */
    public Pose2d goal() {
		return new Pose2d(goalOpt.orElse(Translation2d.kZero), Rotation2d.kZero);
	}

	/**
	 * Calculates attraction force to goal
	 * @param curLocation
	 * @param 
	 * @return
	 */
    Force getGoalForce(Translation2d curLocation, Translation2d goal) {
		var displacement = goal.minus(curLocation);
		if (displacement.getNorm() == 0) {
			return new Force();
		}
		var direction = displacement.getAngle();
		var mag =
			GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
		return new Force(mag, direction);
	}

	/**
	 * Calculates the force acting on the robot from the walls.
	 * @param curLocation Current location of the robot on the field
	 * @param target Target position of the robot
	 * @return force acting on the robot from the walls.
	 */
    Force getWallForce(Translation2d curLocation, Translation2d target) {
		var force = Force.kZero;
		for (Obstacle obs : wallObstacles) {
			force = force.plus(obs.getForceAtPosition(curLocation, target));
		}
		return force;
	}

	/**
	 * Calculates the force acting on the robot from all obstacles.
	 * @param curLocation Current robot position on the field
	 * @param target Target position of the robot
	 * @return Net force acting on the robot from all obstacles.
	 */
    Force getObstacleForce(Translation2d curLocation, Translation2d target) {
		var force = Force.kZero;
		for (Obstacle obs : allFieldObstacles) {
			force = force.plus(obs.getForceAtPosition(curLocation, target));
		}
		return force;
	}

	/**
	 * Calculates the force acting on the robot from all obstacles.
	 * @param curLocation Current robot position on the field
	 * @param target Target position of the robot
	 * @return Net force acting on the robot from all obstacles.
	 */
    Force getForce(Translation2d curLocation, Translation2d target) {
		var goalForce = getGoalForce(curLocation, target)
			.plus(getObstacleForce(curLocation, target))
			.plus(getWallForce(curLocation, target));
		return goalForce;
	}

	/**
	 * Generates a SwerveSample object based on the given parameters.
	 */
    private SwerveSample sample(
		Translation2d trans,
		Rotation2d rot,
		double vx,
		double vy,
		double omega
	) {
		return new SwerveSample(
			0,
			trans.getX(),
			trans.getY(),
			rot.getRadians(),
			vx,
			vy,
			omega,
			0,
			0,
			0,
			new double[4],
			new double[4]
		);
	}

	/**
	 * Sets the goal for the robot.
	 * @param goal Goal position for the robot to move towards.
	 * <p> If the goal is set to an empty optional, the robot will not move towards any goal.
	 * MAKE SURE TO SET THE GOAL BEFORE CALLING getCmd()!
	 */
    public void setGoal(Translation2d goal) {
		this.goalOpt = Optional.of(goal);
		if (simulateArrows) updateArrows();
	}

	/**
	 * Calculates a robot drivetrain in order to reach goal point.
	 * MAKE SURE TO SET THE GOAL BEING USED BEFORE CALLING THIS METHOD!
	 * 
	 * @param pose Current pose of the robot on the field
	 * @param currentSpeeds current robot relative ChassissSpeeds of the robot
	 * @param maxSpeed Max speed of the robot in meters per second
	 * @param useGoal If true, the robot will move towards the goal position.
	 * @return SwerveSample object that contains a ChassissSpeeds that can be extracted to run the drivetrain.
	 */
    public SwerveSample getCmd(
		Pose2d currentPose,
		ChassisSpeeds currentSpeeds,
		double maxSpeed,
		boolean useGoal
	) {
		return getCmd(currentPose, currentSpeeds, maxSpeed, useGoal, currentPose.getRotation());
		
	}

	/**
	 * Calculates a robot drivetrain in order to reach goal point.
	 * MAKE SURE TO SET THE GOAL BEING USED BEFORE CALLING THIS METHOD!
	 * 
	 * @param pose Current pose of the robot on the field
	 * @param currentSpeeds current robot relative ChassissSpeeds of the robot
	 * @param maxSpeed Max speed of the robot in meters per second
	 * @param useGoal If true, the robot will move towards the goal position.
	 * @param goalRotation The desired rotation of the robot when it reaches the goal.
	 * @return SwerveSample object that contains a ChassissSpeeds that can be extracted to run the drivetrain.
	 */
    public SwerveSample getCmd(
		Pose2d pose,
		ChassisSpeeds currentSpeeds,
		double maxSpeed,
		boolean useGoal,
		Rotation2d goalRotation
	) {
		Translation2d speedPerSec = new Translation2d(
			currentSpeeds.vxMetersPerSecond,
			currentSpeeds.vyMetersPerSecond
		);
		double currentSpeed = Math.hypot(
			currentSpeeds.vxMetersPerSecond,
			currentSpeeds.vyMetersPerSecond
		);
		double stepSize_m = maxSpeed * 0.02; // TODO
		if (goalOpt.isEmpty()) {
			return sample(pose.getTranslation(), pose.getRotation(), 0, 0, 0);
		} else {
			var startTime = System.nanoTime();
			var goal = goalOpt.get();
			var curTrans = pose.getTranslation();
			var err = curTrans.minus(goal);
			if (useGoal && err.getNorm() < stepSize_m * 1.5) {
				return sample(goal, goalRotation, 0, 0, 0);
			} else {
				var obstacleForce = getObstacleForce(curTrans, goal).plus(
					getWallForce(curTrans, goal)
				);
				var netForce = obstacleForce;
				if (useGoal) {
					netForce = getGoalForce(curTrans, goal).plus(netForce);
					SmartDashboard.putNumber("forceLog", netForce.getNorm());
					// Calculate how quickly to move in this direction
					var closeToGoalMax = maxSpeed * Math.min(err.getNorm() / 2, 1);

					stepSize_m = Math.min(maxSpeed, closeToGoalMax) * 0.02;
				}
				var step = new Translation2d(stepSize_m, netForce.getAngle());
				var intermediateGoal = curTrans.plus(step);
				var endTime = System.nanoTime();
				SmartDashboard.putNumber("repulsorTimeS", (endTime - startTime) / 1e9);
				return sample(
					intermediateGoal,
					goalRotation,
					step.getX() / 0.02,
					step.getY() / 0.02,
					0
				);
			}
		}
	}

	public double pathLength = 0;

	public ArrayList<Translation2d> getTrajectory(
		Translation2d current,
		Translation2d goalTranslation,
		double stepSize_m
	) {
		pathLength = 0;
		//goalTranslation = goalOpt.orElse(goalTranslation);
		ArrayList<Translation2d> traj = new ArrayList<>();
		Translation2d robot = current;
		for (int i = 0; i < 400; i++) {
			var err = robot.minus(goalTranslation);
			if (err.getNorm() < stepSize_m * 1.5) {
				traj.add(goalTranslation);
				break;
			} else {
				var netForce = getForce(robot, goalTranslation);
				if (netForce.getNorm() == 0) {
					break;
				}
				var step = new Translation2d(stepSize_m, netForce.getAngle());
				var intermediateGoal = robot.plus(step);
				traj.add(intermediateGoal);
				pathLength += stepSize_m;
				robot = intermediateGoal;
			}
		}
		return traj;
	}

	/**
	 * Returns the list of arrows that are used to visualize the repulsor field.
	 * @return List of arrows that represent the repulsor field.
	 */
    public ArrayList<Pose2d> getArrows() {
		return arrows;
	}

	/**
	 * Returns the list of all obstacles in the field.
	 * <p> Used for logging purposes
	 * @return List of all obstacles in the field.
	 */
    public List<Obstacle> getAllFieldObstacles() {
		return allFieldObstacles;
	}
}