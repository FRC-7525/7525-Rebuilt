package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants.Controllers;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIO.IntakeIOOutputs outputs = new IntakeIO.IntakeIOOutputs();
	private double prevTime = 0;
	private boolean agitatingIn = false;
	private boolean allowAutonomousAgitation = false;

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
		//TODO: Find better implementation this is kinda geeked
		if (DRIVER_CONTROLLER.getRightTriggerAxis() > Controllers.TRIGGERS_REGISTER_POINT || OPERATOR_CONTROLLER.getRightTriggerAxis() > Controllers.TRIGGERS_REGISTER_POINT || allowAutonomousAgitation) {
			if (getStateTime() - prevTime > AGITATION_TIME) {
				agitatingIn = !agitatingIn;
				prevTime = getStateTime();
			}

			if (agitatingIn) io.setAngularPosition(INTAKE_AGITATING_IN_POS);
			else io.setAngularPosition(INTAKE_AGITATING_OUT_POS);

			io.setSpinVelocity(SPIN_SPEED_INTAKE);
		} else {
			io.setSpinVelocity(getState().getSpinSpeed());
			io.setAngularPosition(getState().getAngle());
		}

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

	public TalonFX getPivotMotor() {
		return io.getPivotMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}

	public void setAllowAutonomousAgitation(boolean allow) {
		this.allowAutonomousAgitation = allow;
	}
}
