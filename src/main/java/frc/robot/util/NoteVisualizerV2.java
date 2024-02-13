package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.NotePositions;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizerV2 {
  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static final Transform3d launcherTransform =
      new Transform3d(-0.03, 0, 0.54, new Rotation3d(0.0, Units.degreesToRadians(53.0), 0.0));
  private static final Transform2d[] intakeBounds = {
    new Transform2d(Units.inchesToMeters(24), Units.inchesToMeters(-14), new Rotation2d()),
    new Transform2d(Units.inchesToMeters(24), Units.inchesToMeters(14), new Rotation2d())
  };
  private static final double shotSpeed = 5; // Meters per sec
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  private Supplier<Boolean> intakeStateSupplier = () -> false;
  private Consumer<Boolean> noteDetectedSupplier = (Boolean noteDetected) -> {};

  private ArrayList<Pose3d> notePositions = new ArrayList<Pose3d>();

  public NoteVisualizerV2(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Boolean> intakeStateSupplier,
      Consumer<Boolean> noteDetectedSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.intakeStateSupplier = intakeStateSupplier;
    this.noteDetectedSupplier = noteDetectedSupplier;

    stageNotes();
  }

  private Pose3d[] getNotes() {
    Pose3d[] noteArray = new Pose3d[notePositions.size()];
    noteArray = notePositions.toArray(noteArray);
    return noteArray;
  }

  private void stageNotes() {
    for (Pose3d note : NotePositions.kNotesStartingBlueWing) {
      notePositions.add(note);
    }
    for (Pose3d note : NotePositions.kNotesStartingRedWing) {
      notePositions.add(note);
    }
    for (Pose3d note : NotePositions.kNotesStartingMidline) {
      notePositions.add(note);
    }
  }

  public void updateNotes() {

    Pose2d[] AdjustedIntakeBounds = new Pose2d[2];

    for (int i = 0; i < intakeBounds.length; i++) {
      AdjustedIntakeBounds[i] = robotPoseSupplier.get().transformBy(intakeBounds[i]);
    }

    double x1 = AdjustedIntakeBounds[0].getX();
    double y1 = AdjustedIntakeBounds[0].getY();
    double x2 = AdjustedIntakeBounds[1].getX();
    double y2 = AdjustedIntakeBounds[1].getY();

    double a = y2 - y1;
    double b = x1 - x2;
    double c = y1 * (x2 - x1) - (y2 - y1) * x1;

    for (int i = 0; i < notePositions.size(); i++) {

      double x = notePositions.get(i).getX();
      double y = notePositions.get(i).getY();

      double distance = Math.abs(a * x + b * y + c) / Math.sqrt(a * a + b * b);

      double robotDistance =
          robotPoseSupplier
              .get()
              .getTranslation()
              .getDistance(notePositions.get(i).getTranslation().toTranslation2d());

      if (distance < Units.inchesToMeters(14d / 2)
          && robotDistance < Units.inchesToMeters(55.5 / 2)
          && intakeStateSupplier.get()) {
        notePositions.remove(notePositions.get(i));
        noteDetectedSupplier.accept(true);
      }
    }

    Logger.recordOutput("Notes", getNotes());
  }
}
