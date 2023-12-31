package frc.robot;

import frc.robot.consoles.Logger;
import frc.robot.commands.auto.*;
import frc.robot.commands.forklift.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.swervedrive.*;
import frc.robot.commands.limelight.*;

public class BotCommands {

    // Auto Command options
    public static CommandBase placeCubeInner;
    public static CommandBase placeCubeLeft;
    public static CommandBase placeCubeRight;
    public static CommandBase ejectCubeInner;
    public static CommandBase ejectCubeLeft;
    public static CommandBase ejectCubeRight;
    public static CommandBase defaultAutoCommand;

    // SwerveDriver
    public static SwerveDrive swerveDrive;
    public static ToggleDriverOrientation toggleDriverOrientation;

    public static CommandBase toggleOrientation;
    public static CommandBase lockWheels;

    public static CommandBase resetAbsoluteEncoderOffsets;

    public static CalibrateGyro calibrateGyro;

    // Forklift
    public static MoveForklift moveForklift;
    public static QuickForklift quickForklift;
    public static CommandBase resetEncoders;

    //Intake
    public static CommandBase moveIntake;
    public static CommandBase timedIntake;

    // Limelight
    public static EnterZone enterZone;
    public static AlignGyro alignGyro;
    public static AutoDropOff autoDropOff;
    public static AlignLimelight alignLimelight;

    // Initialize all robot commands
    public static void initializeCommands() {

        Logger.setup("Initializing BotCommands...");

        // SwerveDriver
        swerveDrive = new SwerveDrive(BotSubsystems.swerveDriver, BotControllers.xbox1); 
        toggleDriverOrientation = new ToggleDriverOrientation(BotSubsystems.swerveDriver);

        toggleOrientation = BotSubsystems.swerveDriver.toggleOrientationCommand();
        lockWheels = BotSubsystems.swerveDriver.lockWheelsCommand();

        resetAbsoluteEncoderOffsets = BotSubsystems.swerveDriver.resetAbsoluteEncoderOffsets();
        
        calibrateGyro = new CalibrateGyro();

        // Limelight
        alignGyro = new AlignGyro(BotSubsystems.swerveDriver, 0);
        enterZone = new EnterZone(BotSubsystems.swerveDriver);
        autoDropOff = new AutoDropOff(8);
        alignLimelight = new AlignLimelight(BotSubsystems.swerveDriver, 6);
 
        // Forklift
        moveForklift = new MoveForklift(BotSubsystems.forklift); 
        quickForklift = new QuickForklift(BotSubsystems.forklift);

        resetEncoders = BotSubsystems.forklift.resetEncoders();
        

        //Intake not wired yet
        /*
        moveIntake = BotSubsystems.intake.runIntakeCommand();
        timedIntake = BotSubsystems.intake.runIntakeTime(1.0);
        */

        // Auto Commands 
        placeCubeInner = new AutoFactory("Center", "Top");
        placeCubeLeft = new AutoFactory("Left", "Top");
        placeCubeRight = new AutoFactory("Right", "Top");
        ejectCubeInner = new AutoFactory("Center", "Bottom");
        ejectCubeLeft = new AutoFactory("Left", "Bottom");
        ejectCubeRight = new AutoFactory("Right", "Bottom");
        defaultAutoCommand = new DefaultAutoCommand();
    }
}
