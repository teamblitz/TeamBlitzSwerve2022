/* Big thanks to  */

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor_ctre.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.set(percentOutput);
            // TODO This needs to be in rev format.
        }
        else {
            mDriveMotor.getPIDController().setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
            // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // mAngleMotor_ctre.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(mAngleMotor.getEncoder().getPosition());
        // return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor_ctre.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private void resetToAbsolute(){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor_ctre.setSelectedSensorPosition(absolutePosition);
        mAngleMotor.getEncoder().setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
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
        
        mAngleMotor.getEncoder().setPositionConversionFactor(Constants.Swerve.chosenModule.angleGearRatio/360);
        resetToAbsolute();

        mAngleMotor.getPIDController().setP(Constants.Swerve.angleKP);
        mAngleMotor.getPIDController().setI(Constants.Swerve.angleKI);
        mAngleMotor.getPIDController().setD(Constants.Swerve.angleKD);
        mAngleMotor.getPIDController().setFF(Constants.Swerve.angleKF);
    }

    private void configDriveMotor(){       
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveSmartCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(Constants.Swerve.driveSecondaryCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        mDriveMotor.getEncoder().setVelocityConversionFactor(Constants.Swerve.chosenModule.driveGearRatio * Constants.Swerve.chosenModule.wheelCircumference / 60);
        mDriveMotor.getEncoder().setPosition(0);

        mDriveMotor.getPIDController().setP(Constants.Swerve.driveKP);
        mDriveMotor.getPIDController().setI(Constants.Swerve.driveKI);
        mDriveMotor.getPIDController().setD(Constants.Swerve.driveKD);
        mDriveMotor.getPIDController().setFF(Constants.Swerve.driveKF); // Not actually used because we specify our feedforward when we set our speed.
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            // Conversions.falconToMPS(mDriveMotor_ctre.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
            mDriveMotor.getEncoder().getVelocity(), 
            getAngle()
        ); 
    }
}