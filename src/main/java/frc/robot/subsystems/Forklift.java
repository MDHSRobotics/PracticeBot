package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.consoles.Logger;
import com.revrobotics.CANSparkMax.IdleMode;
import java.util.Map;

public class Forklift extends GenericSubsystem {


    public Forklift() {
        super(Map.of("sparkMaxElevator", 3, "sparkMaxArmLift", 9, "sparkMaxClawLift", 8));
        Logger.setup("Constructing Subsystem: Forklift...");

        super.setCANSparkMaxBrakeMode("Elevator", IdleMode.kBrake);
        super.setCANSparkMaxBrakeMode("ArmLift", IdleMode.kBrake);
        super.setCANSparkMaxBrakeMode("ClawLift", IdleMode.kBrake);
    }

    @Override
    public void periodic() {
    }

    //Reset the encoders
    private void resetMotorEncoders(){
        Logger.info("Resetting Motor Encoders");
        super.resetEncoder("Elevator");
        super.resetEncoder("ArmLift");
        super.resetEncoder("ClawLift");
    }

    /* One Line Commands */

    //Reset the motor encoders
    public CommandBase resetEncoders() {
        Logger.info("Resetting Soft Stop");
        return this.runOnce(() -> resetMotorEncoders());
    }
    
}