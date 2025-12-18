// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  Pose2d loadingPose; // This is needed because the april tag field causes a like 6-10 second loop
                      // overrun the first time we see an april tag so we need to run it on boot
  CommandJoystick m_driveJoystick;

  NetworkTable m_leftTable;
  NetworkTable m_rightTable;
  private final DoubleArraySubscriber m_leftSub;
  private final DoubleArraySubscriber m_rightSub;
  private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  Optional<VisionMeasurement> m_leftVisionData = Optional.empty();
  Optional<VisionMeasurement> m_rightVisionData = Optional.empty();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> publisher;
  StructPublisher<Pose2d> m_leftPosePublisher;
  StructPublisher<Pose2d> m_rightPosePublisher;

  public class VisionMeasurement {
    public Pose2d m_visionPose;
    public double m_timeStamp;

    public VisionMeasurement(Pose2d visionPose, double timeStamp, int targetAprilTagID, double angleDiff) {
      m_visionPose = visionPose;
      m_timeStamp = timeStamp;
    }
  }

  public LimelightSubsystem() {
    m_leftTable = NetworkTableInstance.getDefault().getTable("limelight-shpwrte");
    m_rightTable = NetworkTableInstance.getDefault().getTable("limelight-kite");
    m_leftTable.getEntry("imumode_set").setDouble(0);
    m_rightTable.getEntry("imumode_set").setDouble(0); // 3 is scary
    m_leftTable.getEntry("imuassistalpha_set").setDouble(LimelightConstants.imuAssist);
    m_rightTable.getEntry("imuassistalpha_set").setDouble(LimelightConstants.imuAssist);
    m_leftPosePublisher = m_leftTable.getStructTopic("Pose", Pose2d.struct).publish();
    m_rightPosePublisher = m_rightTable.getStructTopic("Pose", Pose2d.struct).publish();
    m_leftSub = m_leftTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);
    m_rightSub = m_rightTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);

    loadingPose = LimelightConstants.aprilTagList[1];
  }

  // TODO: move all geometry code to a geometry util file
  public double getDistanceToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();

    return Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
  }

  public double getAngleToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();
    double angleRadians = Math.atan2(distY, distX);

    return Math.toDegrees(angleRadians);
  }

  public void setThrottle(int throttle) {
    m_leftTable.getEntry("throttle_set").setDouble(throttle);
    m_rightTable.getEntry("throttle_set").setDouble(throttle);
  }

  public void setLEDs(int LEDMode) {
    NetworkTableInstance.getDefault().getTable("limelight-delight").getEntry("ledMode").setNumber(LEDMode);
  }

  public static Pose2d[] getFieldTags(AprilTagFieldLayout field) {
    Pose2d[] output = new Pose2d[field.getTags().size() + 1];
    for (int i = 0; i <= field.getTags().size(); i++) {
      output[i] = field.getTagPose(i).isPresent() ? field.getTagPose(i).get().toPose2d() : new Pose2d();
    }
    return output;
  }

  public Optional<VisionMeasurement> getLimelightData(NetworkTable limelightTable, DoubleArraySubscriber botPoseSub) {
    Optional<VisionMeasurement> output = Optional.empty();

    double[] robotOrientationEntries = new double[6];
    robotOrientationEntries[0] = 0;
    robotOrientationEntries[1] = 0;
    robotOrientationEntries[2] = 0;
    robotOrientationEntries[3] = 0;
    robotOrientationEntries[4] = 0;
    robotOrientationEntries[5] = 0;

    limelightTable.getEntry("robot_orientation_set").setDoubleArray(robotOrientationEntries);

    double[] botPose = null;
    double latency = 0;
    double timeStamp = 0;

    for (TimestampedDoubleArray tsValue : botPoseSub.readQueue()) {
      botPose = tsValue.value;
      if ((int) botPose[7] == 0) {
        continue;
      }
      timeStamp = tsValue.timestamp;
      double botPosX = botPose[0];
      double botPosY = botPose[1];
      // double botPosZ = botpose[2];
      // double botRotX = botpose[3];
      // double botRotY = botpose[4]; // TODO: Store botPosZ, botRotX, and botRotY
      // somewhere (Store 3D)
      double botRotZ = botPose[5];
      latency = botPose[6];
      int targetAprilTagID = (int) botPose[11];

      double adjustedTimeStamp = (timeStamp / 1000000.0) - (latency / 1000.0);

      // timeStamp -= latency / 1000;
      Pose2d visionPose = new Pose2d(botPosX, botPosY, new Rotation2d(Math.toRadians(botRotZ)));
      if (m_poseEstimator != null && m_poseEstimator.getPose() != null && 0 <= targetAprilTagID
          && targetAprilTagID < LimelightConstants.aprilTagList.length) {
        limelightTable.putValue("botPosX", NetworkTableValue.makeDouble(botPosX));
        limelightTable.putValue("botPosY", NetworkTableValue.makeDouble(botPosY));
        limelightTable.putValue("botPosZ", NetworkTableValue.makeDouble(botRotZ));
        if (botPosX != 0 || botPosY != 0 || botRotZ != 0) {
          Rotation2d aprilTagToRobotAngle = Rotation2d.fromDegrees(
              getAngleToPose(visionPose, LimelightConstants.aprilTagList[targetAprilTagID]) - 180);
          limelightTable.putValue("Angle to pose", NetworkTableValue
              .makeDouble(getAngleToPose(visionPose, LimelightConstants.aprilTagList[targetAprilTagID]) - 180));
          double angleDiff = Math.abs(LimelightConstants.aprilTagList[targetAprilTagID].getRotation()
              .minus(aprilTagToRobotAngle)
              .getRadians());
          // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          output = Optional.of(new VisionMeasurement(visionPose, adjustedTimeStamp, targetAprilTagID, angleDiff));
        }
      }
    }
    return output;
  }

  @Override
  public void periodic() {
    m_leftVisionData = getLimelightData(m_leftTable, m_leftSub);
    m_rightVisionData = getLimelightData(m_rightTable, m_rightSub);

    if (m_leftVisionData.isPresent()) {
      VisionMeasurement p = m_leftVisionData.get();
      // m_leftTable.putValue("score", NetworkTableValue.makeDouble(p.m_score));

      // m_leftTable.putValue("unweighted skew",
      // NetworkTableValue.makeDouble(p.m_angleDiff));
      // m_leftTable.putValue("weighted skew",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.skewWeight *
      // p.m_angleDiff));
      // m_leftTable.putValue("unweighted heading",
      // NetworkTableValue.makeDouble(Math.abs(m_poseEstimator.getPose().getRotation().getRadians()
      // - p.m_visionPose.getRotation().getRadians())));
      // m_leftTable.putValue("weighted heading",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.headingWeight
      // * Math.abs(m_poseEstimator.getPose().getRotation().getRadians()
      // - p.m_visionPose.getRotation().getRadians())));
      // m_leftTable.putValue("unweighted distance",
      // NetworkTableValue.makeDouble(p.m_tagDistance));
      // m_leftTable.putValue("weighted distance",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.distanceWeight *
      // p.m_tagDistance));

      m_leftPosePublisher.set(p.m_visionPose);
    }

    if (m_rightVisionData.isPresent()) {
      VisionMeasurement p = m_rightVisionData.get();
      // m_rightTable.putValue("score", NetworkTableValue.makeDouble(p.m_score));

      // m_rightTable.putValue("unweighted skew",
      // NetworkTableValue.makeDouble(p.m_angleDiff));
      // m_rightTable.putValue("weighted skew",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.skewWeight *
      // p.m_angleDiff));
      // m_rightTable.putValue("unweighted heading",
      // NetworkTableValue.makeDouble(Math.abs(m_poseEstimator.getPose().getRotation().getRadians()
      // - p.m_visionPose.getRotation().getRadians())));
      // m_rightTable.putValue("weighted heading",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.headingWeight
      // * Math.abs(m_poseEstimator.getPose().getRotation().getRadians()
      // - p.m_visionPose.getRotation().getRadians())));
      // m_rightTable.putValue("unweighted distance",
      // NetworkTableValue.makeDouble(p.m_tagDistance));
      // m_rightTable.putValue("weighted distance",
      // NetworkTableValue.makeDouble(PoseEstimatorConstants.distanceWeight *
      // p.m_tagDistance));

      m_rightPosePublisher.set(p.m_visionPose);
    }

    if (m_leftVisionData.isPresent() && m_rightVisionData.isPresent()) {
      VisionMeasurement left = m_leftVisionData.get();
      VisionMeasurement right = m_rightVisionData.get();
      double leftScore = left.m_score;
      double rightScore = right.m_score;

      if (leftScore < rightScore) {
        m_poseEstimator.updateVision(left.m_visionPose, left.m_timeStamp, left.m_tagDistance, left.m_angleDiff);
      } else {
        m_poseEstimator.updateVision(right.m_visionPose, right.m_timeStamp, right.m_tagDistance, right.m_angleDiff);
      }
      // factor 1: facing tag, factor 2: matches our robot's angle, factor 3: tag dist
    } else if (m_leftVisionData.isPresent()) {
      VisionMeasurement p = m_leftVisionData.get();
      m_poseEstimator.updateVision(p.m_visionPose, p.m_timeStamp, p.m_tagDistance, p.m_angleDiff);
    } else if (m_rightVisionData.isPresent()) {
      VisionMeasurement p = m_rightVisionData.get();
      m_poseEstimator.updateVision(p.m_visionPose, p.m_timeStamp, p.m_tagDistance, p.m_angleDiff);
    }
  }
}