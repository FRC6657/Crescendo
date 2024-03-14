// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraInformation;
import frc.robot.Constants.VisionConstants.CameraResult;
import frc.robot.Robot;
import java.io.IOException;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {

  private AprilTagCamera backCamera;
  private AprilTagCamera sideCamera;
  //private NoteCamera noteCamera;

  private VisionSystemSim visionSim;

  public Vision(
      CameraInformation backCameraInfo, CameraInformation sideCameraInfo, String noteCameraName) {

    backCamera = new AprilTagCamera(backCameraInfo.name, backCameraInfo.cameraPose);
    sideCamera = new AprilTagCamera(sideCameraInfo.name, sideCameraInfo.cameraPose);

    //noteCamera = new NoteCamera(noteCameraName);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
      visionSim.addCamera(backCamera.getCameraSim(), VisionConstants.kBackCameraPose);
      visionSim.addCamera(sideCamera.getCameraSim(), VisionConstants.kSideCameraPose);
    }
  }

  public CameraResult getBackCameraResult() {
    return backCamera.getCameraResult();
  }

  public CameraResult getSideCameraResult() {
    return sideCamera.getCameraResult();
  }

  public double getNoteX() {
    //return noteCamera.getNoteX();
    return 0;
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public void setFieldTags(Alliance alliance) throws IOException {

    String resource =
        (alliance == Alliance.Blue) ? "/fields/BlueTags.json" : "/fields/RedTags.json";

    AprilTagFieldLayout field = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + resource);

    if (RobotBase.isSimulation()) {
      visionSim.clearAprilTags();
      visionSim.addAprilTags(field);
    }

    backCamera.setAprilTagField(field);
    sideCamera.setAprilTagField(field);

    ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();

    for (int i = 0; i < field.getTags().size(); i++) {
      tagPoses.add(field.getTags().get(i).pose);
    }

    Logger.recordOutput("Vision/Tag Poses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
  }

  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
