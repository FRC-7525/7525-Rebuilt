package frc.robot.Subsystems.Orchestra;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;

import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.Hopper;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class OrchestraSubsystem {
    public Orchestra m_orchestra = new Orchestra();
    private static OrchestraSubsystem instance;

    public static OrchestraSubsystem getInstance() {
		if (instance == null) {
		instance = new OrchestraSubsystem();
		}
		return instance;
	}
    
    private OrchestraSubsystem() {
        // Add a single device to the orchestra
        m_orchestra.addInstrument(Hopper.getInstance().getKickerMotor1());
        m_orchestra.addInstrument(Hopper.getInstance().getKickerMotor2());
        m_orchestra.addInstrument(Hopper.getInstance().getSpinMotor());
        m_orchestra.addInstrument(Intake.getInstance().getSpinMotor());
        m_orchestra.addInstrument(Intake.getInstance().getPivotMotor());
        // m_orchestra.addInstrument(Drive.getInstance().getDriveMotors().get(0));
        // m_orchestra.addInstrument(Drive.getInstance().getDriveMotors().get(1));
        // m_orchestra.addInstrument(Drive.getInstance().getDriveMotors().get(2));
        // m_orchestra.addInstrument(Drive.getInstance().getDriveMotors().get(3));
        // m_orchestra.addInstrument(Drive.getInstance().getTurnMotors().get(0));
        // m_orchestra.addInstrument(Drive.getInstance().getTurnMotors().get(1));
        // m_orchestra.addInstrument(Drive.getInstance().getTurnMotors().get(2));
        // m_orchestra.addInstrument(Drive.getInstance().getTurnMotors().get(3));
        m_orchestra.addInstrument(Shooter.getInstance().getHoodMotor());
        m_orchestra.addInstrument(Shooter.getInstance().getShooterMotors().get(0));
        m_orchestra.addInstrument(Shooter.getInstance().getShooterMotors().get(1));
        
        
        
        AudioConfigs configs = new AudioConfigs().withAllowMusicDurDisable(true);

        // Hopper
        Hopper.getInstance().getKickerMotor1().getConfigurator().apply(configs);
        Hopper.getInstance().getKickerMotor2().getConfigurator().apply(configs);
        Hopper.getInstance().getSpinMotor().getConfigurator().apply(configs);

        // Intake
        Intake.getInstance().getSpinMotor().getConfigurator().apply(configs);
        Intake.getInstance().getPivotMotor().getConfigurator().apply(configs);

        // Drive - Drive Motors
        // Drive.getInstance().getDriveMotors().get(0).getConfigurator().apply(configs);
        // Drive.getInstance().getDriveMotors().get(1).getConfigurator().apply(configs);
        // Drive.getInstance().getDriveMotors().get(2).getConfigurator().apply(configs);
        // Drive.getInstance().getDriveMotors().get(3).getConfigurator().apply(configs);

        // // Drive - Turn Motors
        // Drive.getInstance().getTurnMotors().get(0).getConfigurator().apply(configs);
        // Drive.getInstance().getTurnMotors().get(1).getConfigurator().apply(configs);
        // Drive.getInstance().getTurnMotors().get(2).getConfigurator().apply(configs);
        // Drive.getInstance().getTurnMotors().get(3).getConfigurator().apply(configs);

        // Shooter
        Shooter.getInstance().getHoodMotor().getConfigurator().apply(configs);
        Shooter.getInstance().getShooterMotors().get(0).getConfigurator().apply(configs);
        Shooter.getInstance().getShooterMotors().get(1).getConfigurator().apply(configs);
        
       
      

        // Attempt to load the chrp
        
        var anotheronebitesthedust = m_orchestra.loadMusic("PiratesOfTheCaribbean.chrp");
        // var thunderstrick = m_orchestra.loadMusic("thunderstruck.chrp");
        // var vivaLaVida = m_orchestra.loadMusic("VivaLaVida.chrp");
        // var underTheSea = m_orchestra.loadMusic("UnderTheSea.chrp");
        // var TheFinalCountDown = m_orchestra.loadMusic("TheFinalCountDown.chrp");
        // var RickRolled = m_orchestra.loadMusic("RickRolled.chrp");
        // var PiratesOfTheCaribbean = m_orchestra.loadMusic("PiratesOfTheCaribbean.chrp");
        // var PiratesOfTheCaribbean2 = m_orchestra.loadMusic("PiratesOfTheCaribbean.chrp");
        // var IWantItThatWay = m_orchestra.loadMusic("IWantItThatWay.chrp");
        // var BohimeanRapsody = m_orchestra.loadMusic("BohimeanRapsody.chrp");
        // var BillieJean = m_orchestra.loadMusic("BillieJean.chrp");
    }


    public void playMusic() {
        // Start playing the music
        m_orchestra.play();        
    }

    public void stopMusic() {
        // Stop playing the music
        m_orchestra.stop();
    }

    public boolean isPlaying() {
        Logger.recordOutput("MUSIC/Time", m_orchestra.getCurrentTime());
        return m_orchestra.isPlaying();
    }
}
