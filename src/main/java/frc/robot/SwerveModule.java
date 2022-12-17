/* Big thanks to Team 364 for the base code. */

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle = Rotation2d.fromDegrees(0);

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANCoder absoluteEncoder;

    private RelativeEncoder mDriveEncoder;
    private RelativeEncoder mAngleEncoder;

    private SparkMaxPIDController mDrivePIDController;
    private SparkMaxPIDController mAnglePIDController;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
    private double simSpeedCache;
    private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Encoder */
        absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleEncoder = mAngleMotor.getEncoder();
        mAnglePIDController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive motor */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDrivePIDController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState =
                ModuleStateOptimizer.optimize(
                        desiredState,
                        getState().angle); // Custom optimize command, since default WPILib optimize
        // assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        simSpeedCache = desiredState.speedMetersPerSecond;
        simAngleCache = desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor_ctre.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.set(percentOutput);
        } else {
            mDriveMotor
                    .getPIDController()
                    .setReference(
                            desiredState.speedMetersPerSecond,
                            ControlType.kVelocity,
                            0,
                            feedforward.calculate(desiredState.speedMetersPerSecond));
            // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond,
            // DemandType.ArbitraryFeedForward,
            // feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        // Prevents Jittering.
        // mAngleMotor_ctre.set(ControlMode.Position,
        // Conversions.degreesToFalcon(angle.getDegrees(),
        // Constants.DriveSubsystem.angleGearRatio));
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        if (Robot.isReal()) return Rotation2d.fromDegrees(mAngleEncoder.getPosition());
        return simAngleCache; // If sim.
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    private void resetToAbsolute() {
        mAngleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    private void configAngleEncoder() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleSmartCurrentLimit);
        mAngleMotor.setSecondaryCurrentLimit(Constants.Swerve.angleSecondaryCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);

        mAngleEncoder.setPositionConversionFactor(
                (1
                                / Constants.Swerve.chosenModule
                                        .angleGearRatio) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        mAnglePIDController.setP(Constants.Swerve.angleKP);
        mAnglePIDController.setI(Constants.Swerve.angleKI);
        mAnglePIDController.setD(Constants.Swerve.angleKD);
        mAnglePIDController.setFF(Constants.Swerve.angleKF);

        // TODO: Adjust this latter after we know the pid loop is not crazy
        mAngleMotor.getPIDController().setOutputRange(-.25, .25);
    }

    private void configDriveMotor() {
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveSmartCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(Constants.Swerve.driveSecondaryCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        mDriveEncoder.setVelocityConversionFactor(
                1
                        / Constants.Swerve.chosenModule
                                .driveGearRatio // 1/gear ratio because the wheel spins slower than
                        // the motor.
                        * Constants.Swerve.chosenModule
                                .wheelCircumference // Multiply by the circumference to get meters
                        // per minute
                        / 60); // Divide by 60 to get meters per second.
        mDriveEncoder.setPosition(0);

        mDrivePIDController.setP(Constants.Swerve.driveKP);
        mDrivePIDController.setI(Constants.Swerve.driveKI);
        mDrivePIDController.setD(Constants.Swerve.driveKD);
        mDrivePIDController.setFF(
                Constants.Swerve
                        .driveKF); // Not actually used because we specify our feedforward when we
        // set our speed.

        // TODO: Remove after we know the pid loop isn't wild
        mDrivePIDController.setOutputRange(-.5, .5);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Robot.isReal() ? mDriveEncoder.getVelocity() : simSpeedCache, getAngle());
    }
}
