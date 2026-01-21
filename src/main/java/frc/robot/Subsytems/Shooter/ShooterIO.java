package frc.robot.Subsytems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public AngularVelocity leftWheelVelocity;
        public AngularVelocity rightWheelVelocity;
        public AngularVelocity wheelSetpoint;
        public Angle hoodAngle;
        public Angle hoodSetpoint;
    }

    public abstract void updateInputs(ShooterIOInputs inputs);

    public abstract void setWheelVelocity(AngularVelocity velocity);

    public abstract void setHoodAngle(Angle angle);

    public abstract boolean atWheelVelocitySetpoint();

    public abstract boolean atHoodAngleSetpoint();
}
