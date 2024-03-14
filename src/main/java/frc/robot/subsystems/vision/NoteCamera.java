package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class NoteCamera {

  private PhotonCamera camera;

  public NoteCamera(String name) {
    camera = new PhotonCamera(name);
  }

  public double getNoteX() {
    if (this.hasTarget()) {
      return camera.getLatestResult().getBestTarget().getYaw();
    } else {
      return 0.0;
    }
  }

  public boolean hasTarget() {
    return camera.getLatestResult().hasTargets();
  }
}
