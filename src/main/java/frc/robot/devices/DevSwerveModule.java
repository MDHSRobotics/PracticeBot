package frc.robot.devices;

import frc.robot.subsystems.utils.EncoderTranslator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration; 
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DevSwerveModule {

    private final String m_name;
    private final DevSparkMax m_driveMotor;
    private final DevSparkMax m_turningMotor;

    private final PIDController m_turningPidController;

    private final boolean m_absoluteEncoderReversed;
    private double m_absoluteEncoderOffsetRad;

    private final CANCoder m_canCoder;
    private final EncoderTranslator m_drivingEncoderTranslate;
    private final EncoderTranslator m_turningEncoderTranslate;

    private EncoderTranslator m_encoderTranslator;

    //Constants
    private final double DRIVE_RAMP_TIME = 0.4;
    private final double P_TURNING = 0.5;
    private final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    private final double GEAR_RATIO_DRIVE = 8.16;
    private final double GEAR_RATIO_TURNING = 12.8;

    public DevSwerveModule(String moduleName, DevSparkMax driveSparkMax, DevSparkMax steerSparkMax, CANCoder canCoder,
            boolean driveMotorReversed, boolean turningMotorReversed, boolean absoluteEncoderReversed, double encoderOffset) {

        m_name = moduleName;
        
        m_absoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoderOffsetRad = encoderOffset;

        m_driveMotor = driveSparkMax;
        m_turningMotor = steerSparkMax;

        m_driveMotor.setInverted(driveMotorReversed);
        m_turningMotor.setInverted(turningMotorReversed);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(15);
        m_turningMotor.setSmartCurrentLimit(15);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setIdleMode(IdleMode.kBrake);
        
        m_driveMotor.setOpenLoopRampRate(DRIVE_RAMP_TIME);

        m_turningPidController = new PIDController(P_TURNING, 0, 0);
        m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        m_canCoder = canCoder;
        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_canCoder.setPositionToAbsolute();

        m_drivingEncoderTranslate = new EncoderTranslator("TalonFX", WHEEL_DIAMETER, GEAR_RATIO_DRIVE);

        m_turningEncoderTranslate = new EncoderTranslator("TalonFX", WHEEL_DIAMETER, GEAR_RATIO_TURNING);


        resetEncoders();

        // Group the contents of the module together in SmartDashboard
        //SendableRegistry.setName(m_driveMotor, "Module " + m_name, "Drive Motor");
        //SendableRegistry.setName(m_turningMotor, "Module " + m_name, "Turning Motor");

        m_encoderTranslator = new EncoderTranslator("TalonFX", 1, 12.8);
    }

    public double getDrivePositionMeters() {
        double ticks = m_driveMotor.getEncoder().getPosition();
        double positionInMeters = m_drivingEncoderTranslate.ticks_to_distance(ticks);
        return positionInMeters;
    }

    public double getTurningPositionRadians() {
        double turnTicks = m_turningMotor.getEncoder().getPosition();
        double turnRadians = m_turningEncoderTranslate.ticks_to_radians(turnTicks);
        return turnRadians;
    }

    public double getDriveVelocityMPS() {
        double driveVelocityTPHMS = m_driveMotor.getEncoder().getVelocity();
        double driveVelocityMPS = m_drivingEncoderTranslate.ticksPerDecisecond_to_velocity(driveVelocityTPHMS);
        return driveVelocityMPS;
    }

    public double getTurningVelocityRadiansPS() {
        double turningVelocityTPHMS = m_turningMotor.getEncoder().getVelocity();
        double turningVelocityRPS = m_turningEncoderTranslate.ticksPerDecisecond_to_RadiansPerSecond(turningVelocityTPHMS);
        return turningVelocityRPS;
    }

    public double getAbsoluteEncoderRadians() {
        // Get the angle in degrees of the current position recorded by the absolute encoder
        // Note that this will be -180 to +180, as configured in the constructor above
        double absEncoderPositionDegrees = m_canCoder.getAbsolutePosition();
       
        SmartDashboard.putNumber("Absolute Encoder Raw Angle (Degrees): " + m_name, absEncoderPositionDegrees);
        

        double absEncoderPositionRad = Units.degreesToRadians(absEncoderPositionDegrees);

        //SmartDashboard.putNumber("Absolute Encoder Raw Angle (Radians): " + m_name, absEncoderPositionRad);


        // Apply the offset of the absolute encoder
        absEncoderPositionRad -= m_absoluteEncoderOffsetRad;

        //SmartDashboard.putNumber("Absolute Encoder Angle Without Offset (Radians): " + m_name, absEncoderPositionRad);
        
        // Negate the angle if needed
        return absEncoderPositionRad * (m_absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Get this module's cancoder position in degrees
     * @return the cancoder's absolute position in degrees
     */
    public double getCanCoderAbsolutePosition(){
        return m_canCoder.getAbsolutePosition();
    }

    // Get array of encoder readings:
    // [0] - drive motor (ticks)
    // [1] - drive motor position (meters)
    // [2] - turning motor (ticks)
    // [3] - turning motor angle (degrees)
    public double[] getEncoderReadings() {
        double[] readings = new double[4];

        readings[0] = m_driveMotor.getEncoder().getPosition();
        readings[1] = getDrivePositionMeters();
        readings[2] = m_turningMotor.getEncoder().getPosition();
        readings[3] = getTurningPositionRadians() / Math.PI * 180.;

        return readings;
    }

    public void resetEncoders() {
        // Set drive motor encoder to Zero
        m_driveMotor.getEncoder().setPosition(0.0);

        // Get the initial angle of the absolute encoder in radians
        double initialAbsoluteEncoderPositionRad = getAbsoluteEncoderRadians();

        // Convert angle to raw units (ticks)
        double initialAbsoluteEncoderPositionTicks = m_turningEncoderTranslate.radians_to_ticks(initialAbsoluteEncoderPositionRad);
        
        // Set the current position of the turning motor based on absolute encoder 
        m_turningMotor.getEncoder().setPosition(initialAbsoluteEncoderPositionTicks);

        // Report absolute encoder info to SmartDashboard
        //SmartDashboard.putNumber("Absolute Encoder Angle (Radians): " + m_name, initialAbsoluteEncoderPositionRad);
        //SmartDashboard.putNumber("Absolute Encoder Angle (Ticks): " + m_name, initialAbsoluteEncoderPositionTicks);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMPS(), new Rotation2d(getTurningPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //SmartDashboard.putString("04: Desaturated Desired State: " + m_name, state.toString());

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();

            //SmartDashboard.putString("03: Swerve State: " + m_name, "STOPPED");
            //SmartDashboard.putString("03: Swerve Power: " + m_name, "STOPPED");

            // SmartDashboard.delete("02: Swerve Optimized State: " + m_name);
            // SmartDashboard.delete("01: Swerve Power: " + m_name);

            return;
        } else {
            // SmartDashboard.delete("03: Swerve State: " + m_name);
            // SmartDashboard.delete("03: Swerve Power: " + m_name);
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        //SmartDashboard.putString("02: Swerve Optimized State: " + m_name, state.toString());

        double drivePower = state.speedMetersPerSecond;
        double turningPower = m_turningPidController.calculate(getTurningPositionRadians(), state.angle.getRadians());

        //SmartDashboard.putString("01: Swerve Power: " + m_name, String.format("Drive = %.2f; Turning = %.2f", drivePower, turningPower));

        m_driveMotor.set(drivePower);
        m_turningMotor.set(turningPower);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }

    /*public void setShuffleboardBrain() {
        SwerveDriverBrain.setModuleEncoderReadings(m_name, getEncoderReadings());
    }*/
    
    
    public void setTurningWheelPosition(double angle){  
        m_turningMotor.set(m_encoderTranslator.degrees_to_ticks(angle + Math.toDegrees(m_absoluteEncoderOffsetRad)));
    }

    /*public void setAbsoluteEncoderOffset(){
        Logger.info(String.format("Current encoder offset: %1$s radians and %2$s degrees ", m_absoluteEncoderOffsetRad, Math.toDegrees(m_absoluteEncoderOffsetRad)));
        m_absoluteEncoderOffsetRad = Math.toRadians(SwerveDriverBrain.getAbsoluteEncoderOffset(m_name));
        Logger.info(String.format("Setting encoder offset to %1$s radians and %2$s degrees", m_absoluteEncoderOffsetRad, Math.toDegrees(m_absoluteEncoderOffsetRad)));
    }

    public void resetCanCoder(){
        Logger.info("Cancoder Position" + m_canCoder.getAbsolutePosition());
    }*/

}
