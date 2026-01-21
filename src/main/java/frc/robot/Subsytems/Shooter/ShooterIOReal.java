package frc.robot.Subsytems.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsytems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private TalonFX hoodMotor;
    private Angle hoodSetpoint;
    private AngularVelocity wheelSetpoint;
    private PIDController hoodPID;
    private PIDController wheelPID;
    private SimpleMotorFeedforward wheelFeedforward;

    public ShooterIOReal() {
        leftMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
        rightMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);
        hoodMotor = new TalonFX(HOOD_MOTOR_ID);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned)); // Might need to be inverted
        hoodPID = HOOD_PID.get();
        wheelPID = WHEEL_PID.get();
        wheelFeedforward = WHEEL_FEEDFORWARD.get();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
        inputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
        inputs.wheelSetpoint = wheelSetpoint;
        inputs.hoodAngle = hoodMotor.getPosition().getValue();
        inputs.hoodSetpoint = hoodSetpoint;
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