package frc.robot.Subsytems.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsytems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.GlobalConstants;

public class ShooterIOSim implements ShooterIO {
    private TalonFX leftMotor;
    private TalonFXSimState leftMotorSim;
    private TalonFXSimState rightMotorSim;
    private TalonFXSimState hoodMotorSim;
    private TalonFX rightMotor;
    private TalonFX hoodMotor;
    private Angle hoodSetpoint;
    private AngularVelocity wheelSetpoint;
    private PIDController hoodPID;
    private PIDController wheelPID;
    private SimpleMotorFeedforward wheelFeedforward;
    private FlywheelSim wheelSim;
    private SingleJointedArmSim hoodSim;

    public ShooterIOSim() {
        leftMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
        rightMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);
        hoodMotor = new TalonFX(HOOD_MOTOR_ID);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned)); // Might need to be inverted
        hoodPID = HOOD_PID.get();
        wheelPID = WHEEL_PID.get();
        wheelFeedforward = WHEEL_FEEDFORWARD.get();

        leftMotorSim = new TalonFXSimState(leftMotor);
        rightMotorSim = new TalonFXSimState(rightMotor);
        hoodMotorSim = new TalonFXSimState(hoodMotor);

        wheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                0.5, // Moment of inertia
                1.0 // Gearing
            ), 
            DCMotor.getKrakenX60(2)
        );
        hoodSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(1),
                0.2, // Moment of inertia
                1.0 // Gearing
            ),
            DCMotor.getFalcon500(1),
            1.0, // Arm length
            0, 0, 0,
            true,
            0
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
        inputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
        inputs.wheelSetpoint = wheelSetpoint;
        inputs.hoodAngle = hoodMotor.getPosition().getValue();
        inputs.hoodSetpoint = hoodSetpoint;
        // Sim update
        wheelSim.update(GlobalConstants.SIMULATION_PERIOD);
        hoodSim.update(GlobalConstants.SIMULATION_PERIOD);
        leftMotorSim.setRotorVelocity(wheelSim.getAngularVelocityRadPerSec());
        rightMotorSim.setRotorVelocity(wheelSim.getAngularVelocityRadPerSec());
        hoodMotorSim.setRawRotorPosition(hoodSim.getAngleRads());
        hoodMotorSim.setRotorVelocity(hoodSim.getVelocityRadPerSec());

        // Throw some stuff here for 3D sim later
    }

    @Override
    public void setWheelVelocity(AngularVelocity velocity) {
        wheelSetpoint = velocity;
        leftMotor.set(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSetpoint.in(RotationsPerSecond))
            + wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond)));
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodSetpoint = angle;
        hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValue().in(Degrees), hoodSetpoint.in(Degrees)));
    }

    @Override
    public boolean atWheelVelocitySetpoint() {
        return (Math.abs(leftMotor.getVelocity().getValue().in(RotationsPerSecond) - wheelSetpoint.in(RotationsPerSecond)) < 50) ;
    }

    @Override
    public boolean atHoodAngleSetpoint() {
        return (Math.abs(hoodMotor.getPosition().getValue().in(Degrees) - hoodSetpoint.in(Degrees)) < 0.01);
    }
}