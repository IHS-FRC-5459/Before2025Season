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

  public Matrix<N3, N1> getkMultiTagStdDevs() {
    if (LeftCam.kCameraName.equals(camName)) {
      return LeftCam.kMultiTagStdDevs;
    }
    if (RightCam.kCameraName.equals(camName)) {
      return RightCam.kMultiTagStdDevs;
    }
    return VecBuilder.fill(1000, 1000, 1000);
  }

  public Matrix<N3, N1> getkSingleTagStdDevs() {
    if (LeftCam.kCameraName.equals(camName)) {
      return LeftCam.kSingleTagStdDevs;
    }
    if (RightCam.kCameraName.equals(camName)) {
      return RightCam.kSingleTagStdDevs;
    }
    return VecBuilder.fill(1000, 1000, 1000);
  }

  public Transform3d getkRobotToCam() {
    if (LeftCam.kCameraName.equals(camName)) {
      return LeftCam.kRobotToCam;
    }
    if (RightCam.kCameraName.equals(camName)) {
      return RightCam.kRobotToCam;
    }
    return new Transform3d();
  }

  public static class LeftCam {
    public static final String kCameraName = "left";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.33655, 0.2587625, 0.2778125),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(-45)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class RightCam {
    public static final String kCameraName = "right";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.358775, -0.2682875, 0.27305),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(45)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }
}
