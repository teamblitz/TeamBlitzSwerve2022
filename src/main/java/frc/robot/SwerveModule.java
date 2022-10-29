/* Big thanks to Team 364 for the base code. */

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;

import javax.naming.ldap.ManageReferralControl;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
    private double simSpeedCache;
    private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

//    private Mechanism2d simModule = new Mechanism2d(1,1);
//    MechanismRoot2d simModuleRoot = simModule.getRoot("Center", .5,.5);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
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

//        MechanismObject2d swerveMod = simModuleRoot.append(new MechanismLigament2d("Module", .5,90));
//        swerveMod.
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        simSpeedCache = desiredState.speedMetersPerSecond;
        simAngleCache = desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor_ctre.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.set(percentOutput);
        }
        else {
            mDriveMotor.getPIDController().setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
            // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents Jittering.
        // mAngleMotor_ctre.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        if (Robot.isReal()) return Rotation2d.fromDegrees(mAngleEncoder.getPosition());
        return simAngleCache; // If sim.
        // return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor_ctre.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    private void resetToAbsolute(){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor_ctre.setSelectedSensorPosition(absolutePosition);
        mAngleEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    private void configAngleEncoder(){        
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        // mAngleMotor_ctre.configFactoryDefault();
        // mAngleMotor_ctre.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor_ctre.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor_ctre.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleSmartCurrentLimit);
        mAngleMotor.setSecondaryCurrentLimit(Constants.Swerve.angleSecondaryCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        
        mAngleEncoder.setPositionConversionFactor((1/Constants.Swerve.chosenModule.angleGearRatio) // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
                * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        mAnglePIDController.setP(Constants.Swerve.angleKP);
        mAnglePIDController.setI(Constants.Swerve.angleKI);
        mAnglePIDController.setD(Constants.Swerve.angleKD);
        mAnglePIDController.setFF(Constants.Swerve.angleKF);

        mAngleMotor.getPIDController().setOutputRange(-.25, .25);
    }

    private void configDriveMotor(){       
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveSmartCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(Constants.Swerve.driveSecondaryCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        mDriveEncoder.setVelocityConversionFactor(1/Constants.Swerve.chosenModule.driveGearRatio // 1/gear ratio because the wheel spins slower than the motor.
                * Constants.Swerve.chosenModule.wheelCircumference // Multiply by the circumference to get meters per minute
                / 60); // Devide by 60 to get meters per second.
        mDriveEncoder.setPosition(0);

        mDrivePIDController.setP(Constants.Swerve.driveKP);
        mDrivePIDController.setI(Constants.Swerve.driveKI);
        mDrivePIDController.setD(Constants.Swerve.driveKD);
        mDrivePIDController.setFF(Constants.Swerve.driveKF); // Not actually used because we specify our feedforward when we set our speed.

        mDrivePIDController.setOutputRange(-.5, .5);

    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            // Conversions.falconToMPS(mDriveMotor_ctre.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
            Robot.isReal() ? mDriveEncoder.getVelocity() : simSpeedCache,
            getAngle()
        ); 
    }
}