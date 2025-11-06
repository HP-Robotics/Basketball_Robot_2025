/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  public Optional<Integer> m_sector = Optional.empty();
  public Optional<Integer> m_feederSector = Optional.empty();
  public Translation2d m_targetFeeder;
  SwerveDriveOdometry m_odometry;

  PPHolonomicDriveController m_driveController;

  SwerveSetpointGenerator m_setpointGenerator;
  SwerveSetpoint m_previousSetpoint;

  NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  public NetworkTable m_driveTrainTable = m_inst.getTable("drive-train");
  NetworkTable m_poseEstimatorTable = m_inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> m_posePublisher = m_poseEstimatorTable.getStructTopic("Fused Pose", Pose2d.struct).publish();

  // BIG BONGO 7
  private final SwerveModule m_frontLeft = new SwerveModule(IDConstants.FLDriveMotorID,
      IDConstants.FLTurningMotorID, PortConstants.FLAbsEncoder, DriveConstants.absEncoderForwardFL, "FL");
  // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(IDConstants.FRDriveMotorID,
      IDConstants.FRTurningMotorID, PortConstants.FRAbsEncoder, DriveConstants.absEncoderForwardFR, "FR");
  // BIG BONGO 1
  private final SwerveModule m_backRight = new SwerveModule(IDConstants.BRDriveMotorID,
      IDConstants.BRTurningMotorID, PortConstants.BRAbsEncoder, DriveConstants.absEncoderForwardBR, "BR");
  // BIG BONGO 3
  private final SwerveModule m_backLeft = new SwerveModule(IDConstants.BLDriveMotorID,
      IDConstants.BLTurningMotorID, PortConstants.BLAbsEncoder, DriveConstants.absEncoderForwardBL, "BL");

  private SwerveModuleState[] m_swerveModuleStates = {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  public boolean m_fieldRelative = true;

  public final Field2d m_field = new Field2d();
  public final Field2d m_currentPose = new Field2d();
  public final Field2d m_targetPose = new Field2d();

  private final Pigeon2 m_pGyro = new Pigeon2(IDConstants.PigeonID, "CANivore");
  /*
   * private StatusSignal<Angle> m_pGyroPitch = m_pGyro.getPitch();
   * private StatusSignal<Angle> m_pGyroYaw = m_pGyro.getYaw();
   * private StatusSignal<Angle> m_pGyroRoll = m_pGyro.getRoll();
   */
  private final TrapezoidProfile m_xProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeed / 2.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2.0));
  private final TrapezoidProfile m_yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeed / 2.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2.0));

  PIDController m_rotationController;
  PIDController m_xController;
  PIDController m_yController;
  PoseEstimatorSubsystem m_poseEstimator;

  StructPublisher<Pose2d> m_drivePublisher;
  StructPublisher<Pose2d> m_autoAlignPublisher;
  StructPublisher<Translation2d> joystickTransPub;
  StructPublisher<Translation2d> robotToReefPub;

  public Translation2d joystickTrans;
  public Translation2d robotToReef;
  public Translation2d feederAngle;

  private double m_gyroOffset;
  RobotConfig m_config;

  LEDSubsystem m_ledSubsystem;

  DoubleSupplier m_joystickForward = () -> {
    return Math.signum(ControllerConstants.m_driveJoystick.getRawAxis(1))
        * Math.pow(MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(1),
            ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
        * -1 * DriveConstants.kMaxSpeed;
  };
  DoubleSupplier m_joystickSideways = () -> {
    return Math.signum(ControllerConstants.m_driveJoystick.getRawAxis(0))
        * Math.pow(MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(0),
            ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
        * -1 * DriveConstants.kMaxSpeed;
  };
  DoubleSupplier m_joystickRotation = () -> {
    return MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(4),
        ControllerConstants.driveJoystickDeadband)
        * -1
        * DriveConstants.kMaxAngularSpeed;
  };

  DoubleSupplier m_slowJoystickForward = () -> {
    return Math.signum(ControllerConstants.m_driveJoystick.getRawAxis(1))
        * Math.pow(MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(1),
            ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
        * -1 * DriveConstants.kSlowSpeed;
  };
  DoubleSupplier m_slowJoystickSideways = () -> {
    return Math.signum(ControllerConstants.m_driveJoystick.getRawAxis(0))
        * Math.pow(MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(0),
            ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
        * -1 * DriveConstants.kSlowSpeed;
  };
  DoubleSupplier m_slowJoystickRotation = () -> {
    return MathUtil.applyDeadband(ControllerConstants.m_driveJoystick.getRawAxis(4),
        ControllerConstants.driveJoystickDeadband)
        * -1
        * DriveConstants.kSlowAngularSpeed;
  };
  Function<Double, Double> m_pidX = (Double target) -> {
    return -MathUtil.clamp(m_xController.calculate(m_poseEstimator.getPose().getX(),
        target), -1, 1);
  };
  Function<Double, Double> m_pidY = (Double target) -> {
    return -MathUtil.clamp(m_yController.calculate(m_poseEstimator.getPose().getY(),
        target), -1, 1);
  };
  Function<Rotation2d, Double> m_pidRotation = (Rotation2d targetAngle) -> {
    double output = m_rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        targetAngle.getRadians());
    return m_rotationController.atSetpoint() ? 0 : output;
  };

  public DriveSubsystem(PoseEstimatorSubsystem poseEstimator, LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_pGyro.getYaw().setUpdateFrequency(DriveConstants.odometryUpdateFrequency);

    m_poseEstimator = poseEstimator;
    m_pGyro.setYaw(0);

    try {
      m_config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_currentPose.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_targetPose.setRobotPose(pose);
    });

    // SmartDashboard.putData("Field", m_field);
    m_rotationController = new PIDController(DriveConstants.rotationControllerkP, DriveConstants.rotationControllerkI,
        DriveConstants.rotationControllerkD);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setTolerance(DriveConstants.rotationControllerTolerance);
    m_rotationController.setIZone(DriveConstants.rotationControllerIZone);

    m_xController = new PIDController(DriveConstants.XControllerkP, DriveConstants.XControllerkI,
        DriveConstants.XControllerkD);
    m_xController.setTolerance(DriveConstants.XControllerTolerance);
    m_xController.setIZone(DriveConstants.XControllerIZone);
    m_yController = new PIDController(DriveConstants.YControllerkP, DriveConstants.YControllerkI,
        DriveConstants.YControllerkD);
    m_yController.setTolerance(DriveConstants.YControllerTolerance);
    m_yController.setIZone(DriveConstants.YControllerIZone);

    m_drivePublisher = m_poseEstimatorTable.getStructTopic("Drive Pose", Pose2d.struct).publish();
    m_autoAlignPublisher = m_driveTrainTable.getStructTopic("Auto Align Setpoint", Pose2d.struct).publish();

    if (SubsystemConstants.useDataManager) {
      // SignalLogger.start(); //Phoenix hoot logger
    }

    m_setpointGenerator = new SwerveSetpointGenerator(
        m_config, // The robot configuration
        // This is the same config used for generating trajectories and running path
        // following commands
        Units.rotationsToRadians(DriveConstants.kModuleMaxAngularSpeed)
    // The max rotation velocity of a swerve module in radians per second
    );

    m_previousSetpoint = new SwerveSetpoint(getRobotRelativeSpeeds(), m_swerveModuleStates,
        DriveFeedforwards.zeros(m_config.numModules));

    joystickTransPub = m_driveTrainTable.getStructTopic("Joystick Translation", Translation2d.struct).publish();

    robotToReefPub = m_driveTrainTable.getStructTopic("Robot To Reef", Translation2d.struct).publish();

  }

  public void configureAutoBuilder() {

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards),
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            PathplannerPIDConstants.translationConstants, // Translation PID constants
            PathplannerPIDConstants.rotationConstants // Rotation PID constants
        ),
        m_config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public void updateOdometry() {
    StatusSignal.waitForAll(0,
        m_frontLeft.m_driveMotor.getRotorPosition(),
        m_frontRight.m_driveMotor.getRotorPosition(),
        m_backLeft.m_driveMotor.getRotorPosition(),
        m_backRight.m_driveMotor.getRotorPosition(),
        m_pGyro.getYaw());
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    // Update the odometry in the periodic block
    if (m_poseEstimator != null) {
      m_poseEstimator.updatePoseEstimator(pigeonYaw, new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backRight.getPosition(),
          m_backLeft.getPosition()
      });
    }
  }

  @Override
  public void periodic() {
    if (getPose() != null) {
      m_field.setRobotPose(getPose());
    }

    m_sector = getCurrentSector(m_poseEstimator.getPose());
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      if (m_poseEstimator.getPose().getY() > 4.02) {
        m_feederSector = Optional.of(0);
      } else {
        m_feederSector = Optional.of(1);
      }
    } else {
      if (m_poseEstimator.getPose().getY() > 4.02) {
        m_feederSector = Optional.of(2);
      } else {
        m_feederSector = Optional.of(3);
      }
    }

    // m_driveTrainTable.putValue("Reef sector",
    // NetworkTableValue.makeInteger(m_sector.isPresent() ? m_sector.get() : -1));
    // m_driveTrainTable.putValue("Feeder sector",
    // NetworkTableValue.makeInteger(m_feederSector.isPresent() ?
    // m_feederSector.get() : -1));

    // m_driveTrainTable.putValue("Robot theta",
    // NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("is Drive Abs Working",
        NetworkTableValue.makeBoolean(m_backLeft.m_absEncoder.get() != 0 &&
            m_backRight.m_absEncoder.get() != 0
            && m_frontLeft.m_absEncoder.get() != 0 && m_frontRight.m_absEncoder.get() != 0));

    m_driveTrainTable.putValue("Gyro Angle", NetworkTableValue.makeDouble(getYaw()));

    // TODO investigate why this takes so long
    m_frontLeft.updateShuffleboard();
    m_frontRight.updateShuffleboard();
    m_backRight.updateShuffleboard();
    m_backLeft.updateShuffleboard();

    // BaseStatusSignal.refreshAll(m_pGyroPitch, m_pGyroYaw, m_pGyroRoll);
    // m_driveTrainTable.putValue("Pigeon Pitch",
    // NetworkTableValue.makeDouble(m_pGyroPitch.getValueAsDouble()));
    // m_driveTrainTable.putValue("Pigeon Yaw",
    // NetworkTableValue.makeDouble(m_pGyroYaw.getValueAsDouble()));
    // m_driveTrainTable.putValue("Pigeon Roll",
    // NetworkTableValue.makeDouble(m_pGyroRoll.getValueAsDouble()));

    joystickTrans = new Translation2d(ControllerConstants.m_driveJoystick.getRawAxis(1),
        ControllerConstants.m_driveJoystick.getRawAxis(0));
    robotToReef = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redReefCenter
        : FieldConstants.blueReefCenter).minus(m_poseEstimator.getPose().getTranslation());
    if (m_sector.isPresent()) {
      joystickTransPub.set(new Translation2d(1, 0)
          .rotateBy(new Rotation2d()
              .plus(FieldConstants.leftAlignPoses[m_sector.get()].getRotation().plus(new Rotation2d(Math.PI)))));
      robotToReefPub.set(robotToReef);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the ro bot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));

    m_previousSetpoint = m_setpointGenerator.generateSetpoint(
        m_previousSetpoint, // The previous setpoint
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot), // The desired target speeds
        TimedRobot.kDefaultPeriod // The loop time of the robot code, in seconds
    );
    setModuleStates(m_previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedForwards) {

    m_previousSetpoint = m_setpointGenerator.generateSetpoint(
        m_previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        TimedRobot.kDefaultPeriod // The loop time of the robot code, in seconds
    );
    setModuleStates(m_previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  // TODO: Refactor + get rid of magic numbers
  public void driveWithJoystick(CommandJoystick joystick) {
    drive(
        m_joystickForward.getAsDouble(),
        m_joystickSideways.getAsDouble(),
        m_joystickRotation.getAsDouble(),
        m_fieldRelative);
  }

  public void slowDriveWithJoystick(CommandJoystick joystick) {
    drive(
        m_slowJoystickForward.getAsDouble(),
        m_slowJoystickSideways.getAsDouble(),
        m_slowJoystickRotation.getAsDouble(),
        m_fieldRelative);
  }

  public void drivePointedTowardsAngle(CommandJoystick joystick, Rotation2d targetAngle) {
    double rot = m_pidRotation.apply(targetAngle);
    drive(
        m_joystickForward.getAsDouble(),
        m_joystickSideways.getAsDouble(),
        rot * 1 * DriveConstants.kMaxAngularSpeed,
        m_fieldRelative);

    m_driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(targetAngle.getDegrees()));
    m_driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    m_driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(m_rotationController.getP() * m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(m_rotationController.getSetpoint()));
  }

  public void driveToPose(Pose2d target) {
    int allianceVelocityMultiplier = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    // Alliance velocity multiplier: if it's 1 or -1 based if you're red/blue

    TrapezoidProfile.State m_targetX = m_xProfile.calculate(TimedRobot.kDefaultPeriod,
        new TrapezoidProfile.State(m_poseEstimator.getPose().getX(),
            allianceVelocityMultiplier * getFieldRelativeSpeeds().vxMetersPerSecond),
        new TrapezoidProfile.State(target.getX(), 0.0));

    TrapezoidProfile.State m_targetY = m_yProfile.calculate(TimedRobot.kDefaultPeriod,
        new TrapezoidProfile.State(m_poseEstimator.getPose().getY(),
            allianceVelocityMultiplier * getFieldRelativeSpeeds().vyMetersPerSecond),
        new TrapezoidProfile.State(target.getY(), 0.0));

    m_driveTrainTable.putValue("Real X Error",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getX() - m_xController.getSetpoint()));
    m_driveTrainTable.putValue("Real Y Error",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getY() - m_yController.getSetpoint()));

    double rot = m_pidRotation.apply(target.getRotation());
    // double x = m_pidX.apply(m_targetX.position);
    // double y = m_pidY.apply(m_targetY.position);

    drive(
        allianceVelocityMultiplier * m_targetX.velocity, // x * 1 *
        // DriveConstants.kMaxSpeed,
        allianceVelocityMultiplier * m_targetY.velocity, // y * 1 *
        // DriveConstants.kMaxSpeed,
        rot * DriveConstants.kMaxAngularSpeed,
        true);

    m_autoAlignPublisher.set(target);
    m_driveTrainTable.putValue("X Error", NetworkTableValue.makeDouble(m_xController.getError()));
    m_driveTrainTable.putValue("X P Contribution",
        NetworkTableValue.makeDouble(m_xController.getError() * m_xController.getP()));
    m_driveTrainTable.putValue("X I Contribution",
        NetworkTableValue.makeDouble(m_xController.getAccumulatedError() * m_xController.getI()));
    m_driveTrainTable.putValue("X D Contribution",
        NetworkTableValue.makeDouble(m_xController.getErrorDerivative() * m_xController.getD()));

    m_driveTrainTable.putValue("Theta Error", NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Theta P Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getError() * m_rotationController.getP()));
    m_driveTrainTable.putValue("Theta I Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getAccumulatedError() * m_rotationController.getI()));
    m_driveTrainTable.putValue("Theta D Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getErrorDerivative() * m_rotationController.getD()));

    m_driveTrainTable.putValue("Y Error", NetworkTableValue.makeDouble(m_yController.getError()));
    m_driveTrainTable.putValue("Y P Contribution",
        NetworkTableValue.makeDouble(m_yController.getError() * m_yController.getP()));
    m_driveTrainTable.putValue("Y I Contribution",
        NetworkTableValue.makeDouble(m_yController.getAccumulatedError() * m_yController.getI()));
    m_driveTrainTable.putValue("Y D Contribution",
        NetworkTableValue.makeDouble(m_yController.getErrorDerivative() * m_yController.getD()));
  }

  public void driveForwardWithAngle(double speed, Rotation2d noteAngle) {
    double rot = m_rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        noteAngle.getRadians());
    drive(
        speed * DriveConstants.kMaxSpeed * Math.max(1 - Math.abs(rot), 0),
        0, // speed * -1 * DriveConstants.kMaxSpeed,
        rot * DriveConstants.kMaxAngularSpeed,
        false);

    m_driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(noteAngle.getDegrees()));
    m_driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    m_driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(m_rotationController.getP() * m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(m_rotationController.getSetpoint()));
  }

  public boolean pointedTowardsAngle() {
    return m_rotationController.atSetpoint();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getPose();
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(this.getCurrentSpeeds(), new Rotation2d(Math.toRadians(getYaw())));
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_swerveModuleStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setFieldRelative(boolean isTrue) {
    m_fieldRelative = isTrue;
  }

  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }

  public void startPoseEstimator(Pose2d pose) {
    if (m_poseEstimator != null) {
      m_poseEstimator.createPoseEstimator(DriveConstants.kDriveKinematics,
          new Rotation2d(Math.toRadians(getYaw())), new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backRight.getPosition(),
              m_backLeft.getPosition()
          }, pose);
    }
  }

  public void resetOffsets() { // Turn encoder offset
    m_frontLeft.resetOffset();
    m_frontRight.resetOffset();
    m_backRight.resetOffset();
    m_backLeft.resetOffset();
  }

  public void resetPose(Pose2d pose) {
    if (pose == null) {
      return;
    }
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    m_poseEstimator.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
  }

  public void resetPoseEstimatorHeading(Rotation2d angle) {
    if (angle == null) {
      return;
    }
    Pose2d pose = new Pose2d(m_poseEstimator.getPose().getTranslation(), angle);
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    m_poseEstimator.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
  }

  public boolean arePosesSimilar(Pose2d pose1, Pose2d pose2) {
    return getDistanceToPose(pose1, pose2) <= DriveConstants.poseDistanceTolerance
        && Math.abs(pose1.getRotation().minus(pose2.getRotation()).getRadians()) <= DriveConstants.poseAngleTolerance;
  }

  public double getDistanceToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();

    return Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
  }

  public double getAngleBetweenPoses(Pose2d pose1, Pose2d pose2) {
    double distX = pose2.getX() - pose1.getX();
    double distY = pose2.getY() - pose1.getY();
    double angleRadians = Math.atan2(distY, distX);

    return Math.toDegrees(angleRadians);
  }

  public Optional<Integer> getCurrentSector(Pose2d pose) {
    boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d reefCenter = new Pose2d(isRed ? FieldConstants.redReefCenter : FieldConstants.blueReefCenter,
        new Rotation2d());

    if (getDistanceToPose(reefCenter, pose) > FieldConstants.autoAlignSectorRadius) {
      return Optional.empty();
    }
    double angle = MathUtil.inputModulus(getAngleBetweenPoses(reefCenter, pose) + FieldConstants.autoAlignSectorOffset,
        0, 360);
    int sector = (int) (angle / (360 / FieldConstants.autoAlignSectorCount));
    if (isRed) {
      sector += FieldConstants.autoAlignSectorCount;
    }
    // System.out.println(sector);
    return Optional.of(sector);
  }

  public boolean isNearTargetAngle(double angle, double targetAngle, double tolerance) {
    return Math.abs(angle - targetAngle) <= tolerance;
  }

  public boolean isNearTargetAngle(Translation2d a, Translation2d b, double tolerance) {
    return Math.abs(Math.acos((a.getX() * b.getX() + a.getY() * b.getY())
        / (a.getNorm() * b.getNorm()))) <= tolerance;
  }

  public double getAngleBetweenVectors(Translation2d a, Translation2d b) {
    return Math.acos((a.getX() * b.getX() + a.getY() * b.getY())
        / (a.getNorm() * b.getNorm()));
  }

  public boolean isStopped(ChassisSpeeds speeds, double threshold) {
    return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
        + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond) < threshold;
  }

  public Command AutoAlign(Pose2d[] targetList) {
    return new RunCommand(() -> {
      Pose2d targetPose;
      if (m_sector.isPresent()
          && (isNearTargetAngle(joystickTrans, robotToReef,
              DriveConstants.autoAlignJoystickTolerance)
              || ControllerConstants.m_driveJoystick
                  .getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
        targetPose = targetList[m_sector.get()];

        double distanceToTarget = getDistanceToPose(getPose(), targetPose);
        double theta = getAngleBetweenVectors(
            getPose().getTranslation().minus(targetPose.getTranslation()),
            new Translation2d(1, 0)
                .rotateBy(new Rotation2d().plus(targetPose.getRotation().plus(new Rotation2d(Math.PI)))));
        double distance = distanceToTarget * Math.sin(theta);
        m_driveTrainTable.putValue("theta to target", NetworkTableValue.makeDouble(theta));
        m_driveTrainTable.putValue("horizontal distance to target", NetworkTableValue.makeDouble(distance));
        if (distance < 0.01) {
          distance = 0;
        }

        Pose2d newPose = new Pose2d(
            targetPose.getTranslation()
                .plus(
                    new Translation2d(FieldConstants.autoAlignDistanceMultiplier * distance, 0)
                        .rotateBy(targetPose.getRotation().plus(new Rotation2d(Math.PI)))),
            targetPose.getRotation());
        driveToPose(newPose);
        if (arePosesSimilar(getPose(), targetPose) && isStopped(getCurrentSpeeds(), DriveConstants.velocityThreshold)) {
          LEDSubsystem.trySetSidePattern(m_ledSubsystem, LEDConstants.autoAlignReadyPattern);
        } else {
          LEDSubsystem.trySetSidePattern(m_ledSubsystem, LEDConstants.autoAligningPattern);
        }
      } else {
        drivePointedTowardsAngle(
            ControllerConstants.m_driveJoystick,
            Rotation2d.fromDegrees(getAngleBetweenPoses(getPose(),
                new Pose2d(
                    DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.redReefCenter
                        : FieldConstants.blueReefCenter,
                    new Rotation2d()))));
        LEDSubsystem.trySetSidePattern(m_ledSubsystem, LEDConstants.autoAligningPattern);

      }
      m_driveTrainTable.putValue("Joystick Degrees",
          NetworkTableValue.makeDouble(180 - ControllerConstants.m_driveJoystick.getDirectionDegrees()));
    }, this);
  }

  /**
   * Sets robot's conception of field orientation
   */
  public void setYaw(double angle) {
    m_gyroOffset = m_pGyro.getYaw().getValueAsDouble() - angle;
  }

  public double getYaw() {
    return MathUtil.inputModulus(m_pGyro.getYaw().getValueAsDouble() - m_gyroOffset, -180.0, 180.0);
  }

  public void resetYaw() {
    setYaw(0);
  }

  public Command StayStillCommand() {
    return new InstantCommand(() -> drive(0, 0, 0, true), this);
  }
}