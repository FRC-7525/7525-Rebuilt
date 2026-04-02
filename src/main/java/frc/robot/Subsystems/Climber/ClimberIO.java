package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface ClimberIO {
	class ClimberIOOutputs {

		public Angle angularPos = Rotations.of(0);
		public Angle angularSetpoint = Rotations.of(0);
	}

	void logOutputs(ClimberIOOutputs outputs);

	void setPosition(Angle position);

	boolean atPositionSetpoint();
}
