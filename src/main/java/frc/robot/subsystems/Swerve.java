/* Big thanks to Team 364 for the base code. */

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve");

    private final Field2d field = new Field2d();

    public Swerve() {
        gyro = new AHRS();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] { // front left, front rigt, back left, back right.
            new SwerveModule(FL, Mod0.constants),
            new SwerveModule(FR, Mod1.constants),
            new SwerveModule(BL, Mod2.constants),
            new SwerveModule(BR, Mod3.constants)
        };
        initTelemetry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */ // Use in above method?
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }



    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return(invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        drawRobotOnField(field);
    }

    public void initTelemetry(){
        for(SwerveModule mod : mSwerveMods){
            shuffleboardTab.addNumber("Mod " + mod.moduleNumber + " CanCoder", () -> mod.getCanCoder().getDegrees());
            shuffleboardTab.addNumber("Mod " + mod.moduleNumber + " Integrated", () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber("Mod " + mod.moduleNumber + " Rotation", () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond); 
        }
        shuffleboardTab.add(field);
    }

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        SwerveModuleState[] swerveModuleStates = getModuleStates();
        field.getObject("frontLeft").setPose(
                getPose().transformBy(new Transform2d(robotToModuleTL.get(FL), swerveModuleStates[FL].angle))
        );
        field.getObject("frontRight").setPose(
                getPose().transformBy(new Transform2d(robotToModuleTL.get(FR), swerveModuleStates[FR].angle))
        );
        field.getObject("backLeft").setPose(
                getPose().transformBy(new Transform2d(robotToModuleTL.get(BL), swerveModuleStates[BL].angle))
        );
        field.getObject("backRight").setPose(
                getPose().transformBy(new Transform2d(robotToModuleTL.get(BR), swerveModuleStates[BR].angle))
        );
    }
}