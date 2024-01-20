package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ChuckConstants;


// used for "output" in order to throw note into goal

// TODO: Potentially use limelight to only allow shooting of the ring IF limelight is aligned with the target (requires offsets and knowing where it'll be placed)

public class Chuck extends SubsystemBase{

    // Motors

    // Speaker motors
    private final CANSparkMax speakerMotor1 = new CANSparkMax(ChuckConstants.id10, MotorType.kBrushless);
    private final CANSparkMax speakerMotor2 = new CANSparkMax(ChuckConstants.id11, MotorType.kBrushless);
    // Amp motor
    private final CANSparkMax ampMotor = new CANSparkMax(ChuckConstants.id12, MotorType.kBrushless);

    // Initialize new output
    public Chuck() {

        // By default, motors will be stopped
        speakerMotor1.setIdleMode(IdleMode.kBrake);
        speakerMotor2.setIdleMode(IdleMode.kBrake);
        ampMotor.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public CommandBase SpeakerShoot() {
        return run(
            () -> {
                speakerMotor1.set(ChuckConstants.speakerspeed);
                speakerMotor2.set(ChuckConstants.speakerspeed);
            });
    }
    
    public CommandBase AmpShoot() {
        return run(
            () -> {
                ampMotor.set(ChuckConstants.ampspeed);
            });
    }
}
