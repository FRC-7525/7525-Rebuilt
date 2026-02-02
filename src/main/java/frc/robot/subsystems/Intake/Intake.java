package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIO.IntakeIOOutputs outputs = new IntakeIO.IntakeIOOutputs();
	private final SimpleMotorFeedforward spinFF = new SimpleMotorFeedforward(SPIN_kS, SPIN_kV, SPIN_kA);

	private Intake() {
		super(SUBSYSTEM_NAME, IntakeStates.IN);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new IntakeIOSim();
			case REAL -> new IntakeIOTalonFX();
			case TESTING -> new IntakeIOTalonFX();
		};
	}

	public static Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setLinearPosition(getState().getLinearPos());
		io.setSpinVelocity(getState().getSpinSpeed(), spinFF.calculate(getState().getSpinSpeed().in(RotationsPerSecond)));
		io.logOutputs(outputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinVelocityRPS", outputs.spinVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinSetpointRPS", outputs.spinSetpoint.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinAppliedVolts", outputs.spinAppliedVolts);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinCurrentAmps", outputs.spinCurrentAmps);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearPositionMeters", outputs.linearPosition.in(Meters));
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearSetpointMeters", outputs.linearSetpoint.in(Meters));
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearVelocityMetersPerSec", outputs.linearVelocity.in(MetersPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearAppliedVolts", outputs.linearAppliedVolts);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearCurrentAmps", outputs.linearCurrentAmps);
	}

	public TalonFX getSpinMotor() {
		return io.getSpinMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
