package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraConstants {
  private String camName;

  public CameraConstants(String kCameraName) {
    this.camName = camName;
  }

  public String getkCameraName() {
    if (LeftCam.kCameraName.equals(camName)) {
      return LeftCam.kCameraName;
    }
    if (RightCam.kCameraName.equals(camName)) {
      return RightCam.kCameraName;
    }
    return "";
  }

  public static class LeftCam {
    public static final String kCameraName = "left";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.10795 + 0.3048, 0.21, 0.282575),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(-45)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class RightCam {
    public static final String kCameraName = "left";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.10795 + 0.3048, 0.21, 0.282575),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(-45)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }
}
