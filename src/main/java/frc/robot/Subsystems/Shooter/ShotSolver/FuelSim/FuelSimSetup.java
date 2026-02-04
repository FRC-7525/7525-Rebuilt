package frc.robot.Subsystems.Shooter.ShotSolver.FuelSim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeStates;

public class FuelSimSetup {
    public static void setup() {
        //FuelSim.getInstance().spawnStartingFuel();
        FuelSim.getInstance().registerRobot(
            0.5, // from left to right
            0.5, // from front to back
            0.14, // from floor to top of bumpers
            () -> Drive.getInstance().getPose(), // Supplier<Pose2d> of robot pose
            () -> Drive.getInstance().getFieldCentricSpeeds() // Supplier<ChassisSpeeds> of field-centric chassis speeds
        ); 

        FuelSim.getInstance().registerIntake(
            -0.25,
            0.25, 
            0.25,
            0.45,
            () -> Intake.getInstance().getState() == IntakeStates.INTAKE
        );

        FuelSim.getInstance().enableAirResistance();
        FuelSim.getInstance().setSubticks(1);
        FuelSim.getInstance().start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
    }
}
