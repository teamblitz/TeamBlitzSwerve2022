/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems.drive;

import static frc.robot.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Logger logger;
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("DriveSubsystem");
    private final Field2d field = new Field2d();

    public DriveSubsystem(
            SwerveModuleIO frontLeft,
            SwerveModuleIO frontRight,
            SwerveModuleIO backLeft,
            SwerveModuleIO backRight,
            GyroIO gyroIO) {
        this(
                new SwerveModule(FL, frontLeft),
                new SwerveModule(FR, frontRight),
                new SwerveModule(BL, backLeft),
                new SwerveModule(BR, backRight),
                gyroIO);
    }

    public DriveSubsystem(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            GyroIO gyroIO) {
        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw(), getModulePositions());
        swerveModules =
                new SwerveModule[] { // front left, front right, back left, back right.
                    frontLeft, frontRight, backLeft, backRight
                };
        this.gyroIO = gyroIO;
        logger = Logger.getInstance();
        initTelemetry();
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    //    public void park

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void zeroGyro() {
//        gyro.reset();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyroInputs.yaw);
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : swerveModules) {
            mod.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        logger.processInputs("gyro", gyroInputs);

        swerveOdometry.update(getYaw(), getModulePositions());
    }

    @Override
    public void simulationPeriodic() {
        drawRobotOnField(field);
    }

    public void initTelemetry() {
        for (SwerveModule mod : swerveModules) {
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " CanCoder",
                    () -> mod.getAbsoluteAngle().getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Integrated",
                    () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Rotation",
                    () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Velocity",
                    () -> mod.getState().speedMetersPerSecond);
        }
        shuffleboardTab.add(field);
    }

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module
        // relative to robot center,
        // then rotated around its own center by the angle of the module.
        SwerveModuleState[] swerveModuleStates = getModuleStates();
        field.getObject("frontLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FL),
                                                swerveModuleStates[FL].angle)));
        field.getObject("frontRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FR),
                                                swerveModuleStates[FR].angle)));
        field.getObject("backLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BL),
                                                swerveModuleStates[BL].angle)));
        field.getObject("backRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BR),
                                                swerveModuleStates[BR].angle)));
    }
}
