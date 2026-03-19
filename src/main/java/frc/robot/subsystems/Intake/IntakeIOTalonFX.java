package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Intake.IntakeConstants.Real;

public class IntakeIOTalonFX implements IntakeIO {

	protected final TalonFX spinMotor;
	protected final TalonFXConfiguration spinMotorConfiguration = new TalonFXConfiguration();
	protected final TalonFX pivotMotor;
	protected final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration();
	private final PIDController pivotController;
	private Angle setpoint;
	protected DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);

	public IntakeIOTalonFX() {
		pivotController = PIVOT_PID.get();
		spinMotor = new TalonFX(Real.SPIN_MOTOR_ID);
		spinMotor.setNeutralMode(NeutralModeValue.Coast);
		spinMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
		spinMotor.getConfigurator().apply(spinMotorConfiguration);

		pivotMotor = new TalonFX(Real.PIVOT_MOTOR_ID);
		pivotMotor.setNeutralMode(NeutralModeValue.Coast);
		pivotMotor.setPosition(0);
		setpoint = Degrees.of(0);
	}

	@Override
	public void logOutputs(IntakeIOOutputs outputs) {
		outputs.spinVelocity = spinMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValue();
		outputs.spinCurrentAmps = spinMotor.getSupplyCurrent().getValue();
		outputs.angularPosition = Rotations.of(pivotMotor.getPosition().getValueAsDouble() / GEARING);
		outputs.angularSetpoint = setpoint;
		outputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
		SmartDashboard.putData("PIVOT_CONTROLLER", pivotController);
	}

	@Override
	public void setSpinVelocity(double speed) {
		spinMotor.set(speed);
	}

	@Override
	public void setAngularPosition(Angle setpoint) {
		if (!limitSwitch.get() && !(pivotMotor.getPosition().getValueAsDouble() < TOLERANCE)) { // If the limit switch is pressed and we're not near 0;
			pivotMotor.setPosition(0);
		}
		this.setpoint = setpoint;
		pivotMotor.set(pivotController.calculate(pivotMotor.getPosition().getValue().in(Degrees) / GEARING, setpoint.in(Degrees)));
	}

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}

	@Override
	public TalonFX getPivotMotor() {
		return pivotMotor;
	}
}
