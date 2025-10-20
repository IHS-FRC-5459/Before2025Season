// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  Camera[] cameras;

  Pigeon2 pigeon;
  Drive drive;
  SwerveDrivePoseEstimator swerveEstimator;

  public Vision(String[] cameraNames, Pigeon2 pigeon, Drive drive) {
    // Sets up cameras list
    // I know its sloppy, but it hopefully wors :)
    ArrayList<Camera> camerasList = new ArrayList<Camera>();
    for (String name : cameraNames) {
      camerasList.add(new Camera(name));
    }
    cameras = new Camera[camerasList.size()];
    for (int i = 0; i < camerasList.size(); i++) {
      cameras[i] = camerasList.get(i);
    }

    this.pigeon = pigeon;
    this.drive = drive;

    swerveEstimator =
            new SwerveDrivePoseEstimator(Constants.Vision.kinematics, new Rotation2d(), Constants.Vision.lastModulePositions, new Pose2d());
  }
  private Pose2d fusedPose = new Pose2d();
  private Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(0.7, 0.5, 0.2);

  //To be implimented. Will update fusedPose and fusedStdDevs
  public void fuse(Pose2d[] poses, Matrix<N3, N1>[] stdDevsArray) {
    
  }
  public Pose2d getFusedPose() {
    return fusedPose;
  }

  public Matrix<N3, N1> getFusedStdDevs() {
    return fusedStdDevs;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
      ArrayList<Matrix> stdDevs = new ArrayList<Matrix>();
      for (int i = 0; i < cameras.length; i++) {
        Camera camera = cameras[i];
        camera.periodic();
         if (camera.getLatestLocation() != null
            && camera.getEstStdDevs() != null
            && camera.canSeeTarget()) {
          poses.add(camera.getLatestLocation().toPose2d());
          stdDevs.add(camera.getEstStdDevs());
        }
      }
      Pose2d[] posesArr = poses.toArray(new Pose2d[0]);
      Matrix[] stdDevsArr = stdDevs.toArray(new Matrix[0]);
      swerveEstimator.update(pigeon.getRotation2d(), drive.getModulePositions());
      Logger.recordOutput("photonvisionLogging/pigeonRot", pigeon.getRotation2d().getDegrees());

      if (posesArr.length > 0 && stdDevsArr.length > 0) {
        fuse(posesArr, stdDevsArr);
        swerveEstimator.addVisionMeasurement(
            this.getFusedPose(), Timer.getFPGATimestamp(), this.getFusedStdDevs().times(2));
        Logger.recordOutput("photonvisionLogging/isUpdatingWCameras", true);
      } else {
        Logger.recordOutput("photonvisionLogging/isUpdatingWCameras", false);
      }
      // This method will be called once per scheduler run
      Logger.recordOutput("photonvisionLogging/est Pose", getFusedPose());
  }
}
