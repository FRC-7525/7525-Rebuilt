package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.Sim;

public class IntakeIOSim extends IntakeIOTalonFX {

	private DCMotorSim spinSim;
	private TalonFXSimState spinSimState;
	private double speed;
	private Angle setpoint;
	private DCMotorSim linearSim;
	private TalonFXSimState linearSimState;
	private final PIDController linearController;

	//TODO: Make this not a linear sim and use Krakens instead of falcons
	public IntakeIOSim() {
		speed = 0;
		setpoint = Degrees.of(0);
		spinSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));
		spinSimState = new TalonFXSimState(spinMotor);

		linearSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, GEARING), DCMotor.getFalcon500(1));
		linearSimState = new TalonFXSimState(pivotMotor);
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
		linearSimState.setRotorVelocity((linearSim.getAngularVelocityRPM() / 60) * GEARING);
		linearSimState.setRawRotorPosition(linearSim.getAngularPositionRotations() * GEARING);
		linearSimState.setSupplyVoltage(12.0);

		outputs.spinSetpoint = speed;
		outputs.angularSetpoint = setpoint;
		outputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValue();
		outputs.spinCurrentAmps = spinMotor.getStatorCurrent().getValue();
		// incomplete sim bc buns
	}

	@Override
	public void setSpinVelocity(double speed) {
		this.speed = speed;
		spinSim.setInputVoltage(12);
	}

	@Override
	public void setAngularPosition(Angle setpoint) {
		this.setpoint = setpoint;
		linearSim.setInputVoltage(12 * linearController.calculate(linearSim.getAngularPositionRotations(), setpoint.in(Degrees) / GEARING));
	} // pid only for sim, irl uses talon pid

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}
