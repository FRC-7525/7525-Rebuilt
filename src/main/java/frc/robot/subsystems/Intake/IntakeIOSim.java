package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.Sim;

public class IntakeIOSim extends IntakeIOTalonFX {

	private DCMotorSim spinSim;
	private TalonFXSimState spinSimState;
	private AngularVelocity speed;
	private Distance setpoint;
	private DCMotorSim linearSim;
	private TalonFXSimState linearSimState;
	private final PIDController linearController;

	public IntakeIOSim() {
		speed = RotationsPerSecond.of(0);
		setpoint = Meters.of(0);
		spinSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));
		spinSimState = new TalonFXSimState(spinMotor);

		linearSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, LINEAR_GEARING), DCMotor.getFalcon500(1));
		linearSimState = new TalonFXSimState(linearMotor);
		linearController = new PIDController(Sim.LINEAR_PID.kP, Sim.LINEAR_PID.kI, Sim.LINEAR_PID.kD);
	}

	@Override
	public void logOutputs(IntakeIOOutputs outputs) {
		spinSim.update(GlobalConstants.SIMULATION_PERIOD);
		spinSimState.setRotorVelocity(spinSim.getAngularVelocityRPM() / 60);
		spinSimState.setRawRotorPosition(spinSim.getAngularPositionRotations());
		spinSimState.setSupplyVoltage(12.0);

		//SmartDashboard.putData("Intake TUNER", linearController);

		linearSim.update(GlobalConstants.SIMULATION_PERIOD);
		linearSimState.setRotorVelocity((linearSim.getAngularVelocityRPM() / 60) * LINEAR_GEARING);
		linearSimState.setRawRotorPosition(linearSim.getAngularPositionRotations() * LINEAR_GEARING);
		linearSimState.setSupplyVoltage(12.0);

		outputs.spinVelocity = spinMotor.getVelocity().getValue();
		outputs.spinSetpoint = speed;
		outputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValue();
		outputs.spinCurrentAmps = spinMotor.getStatorCurrent().getValue();

		outputs.linearAppliedVolts = linearMotor.getMotorVoltage().getValue();
		outputs.linearCurrentAmps = linearMotor.getStatorCurrent().getValue();
		outputs.linearPosition = Meters.of(linearSim.getAngularPositionRotations() * LINEAR_METERS_PER_ROTATION);
		outputs.linearSetpoint = setpoint;
		outputs.linearVelocity = MetersPerSecond.of(linearSim.getAngularVelocity().in(RotationsPerSecond) * LINEAR_METERS_PER_ROTATION);

		Logger.recordOutput("Intake/Intake Position", new Pose3d(0.296694, 0.0, 0.223, new Rotation3d(Radians.of(0), Radians.of(linearSim.getAngularPositionRotations()), Radians.of(0))));
	}

	@Override
	public void setSpinVelocity(AngularVelocity speed, double feedforward) {
		this.speed = speed;
		spinSim.setInputVoltage(12 * feedforward);
	}

	@Override
	public void setLinearPosition(Distance setpoint) {
		this.setpoint = setpoint;
		linearSim.setInputVoltage(12 * linearController.calculate(linearSim.getAngularPositionRotations(), setpoint.in(Meters) / LINEAR_METERS_PER_ROTATION));
	} // pid only for sim, irl uses talon pid

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}
