// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraInformation;
import frc.robot.Constants.VisionConstants.CameraResult;
import frc.robot.Robot;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {

  private Camera backCamera;
  private Camera sideCamera;

  private VisionSystemSim visionSim;

  public Vision(CameraInformation backCameraInfo, CameraInformation sideCameraInfo) {

    backCamera = new Camera(backCameraInfo.name, backCameraInfo.cameraPose);
    sideCamera = new Camera(sideCameraInfo.name, sideCameraInfo.cameraPose);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.kTagLayout);

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

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
