package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIO.IntakeIOOutputs outputs = new IntakeIO.IntakeIOOutputs();

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
		io.setSpinVelocity(getState().getSpinSpeed());

		//TODO: Find better implementation this is kinda geeked
		if (getState() == IntakeStates.AGITATING) {
			if (Math.floor(getStateTime()) % 2 == 0) {
				io.setAngularPosition(INTAKE_AGITATING_IN_POS);
			} else io.setAngularPosition(INTAKE_AGITATING_OUT_POS);
		} else io.setAngularPosition(getState().getAngle());

		io.logOutputs(outputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinVelocityRPS", outputs.spinVelocity);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinSetpointRPS", outputs.spinSetpoint);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinAppliedVolts", outputs.spinAppliedVolts);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinCurrentAmps", outputs.spinCurrentAmps);
		Logger.recordOutput(SUBSYSTEM_NAME + "/AngularDegrees", outputs.angularPosition.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/AngularSetpointDegrees", outputs.angularSetpoint.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/PivotCurrentAmps", outputs.pivotCurrentAmps);
	}

	public TalonFX getSpinMotor() {
		return io.getSpinMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
