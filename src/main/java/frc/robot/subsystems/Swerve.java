package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

    GenericEntry allianceEntry = Shuffleboard.getTab("Game").add("Alliance", false).getEntry();
    GenericEntry matchEntry = Shuffleboard.getTab("Game").add("Game Match", "0").getEntry();
    Optional<Alliance> ally = DriverStation.getAlliance();

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    public ChassisSpeeds mSpeeds;
    public SwerveDrivePoseEstimator robotPose;

    // initializes widgets for field/odometry on shuffleboard & swerve angle
    Field2d field = new Field2d();
    ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("2024-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
    GenericEntry swerveAngleEntry = Shuffleboard.getTab("Swerve").add("Swerve Angle", 0).getEntry();
    GenericEntry encoderAngleEntry = Shuffleboard.getTab("Swerve").add("Encoder Angle",0).getEntry();


    // Reporting values for AdvantageScope
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        robotPose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), getPose());


        // PathPlanner AutoBuilder

        // AutoBuilder.configureHolonomic(
        //     this::getPose, // Robot pose supplier
        //     this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants TODO: update this
        //             new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        //             4.5, // Max module speed, in m/s TODO: update this
        //             0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        //             new ReplanningConfig() // Default path replanning config. See the API for the options here
        //     ),
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
        // );
    }

    // initialize pose estimator, needed for odometry autos

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(  
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        mSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // Only robot relative drive mode used for pathplanner

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        mSpeeds = speeds;
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        System.out.println(mSpeeds);
        return mSpeeds;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
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

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading (){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
   
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        
        //System.out.println(RobotContainer.opBoard.getRawAxis(3));
        //System.out.println(getGyroYaw());

        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                allianceEntry.setBoolean(false);
            }
            if (ally.get() == Alliance.Blue) {
                 allianceEntry.setBoolean(true);
            }
        }
        matchEntry.setDouble(DriverStation.getMatchTime());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
            
        

        }
    }
}