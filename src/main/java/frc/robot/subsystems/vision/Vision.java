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
        new SwerveDrivePoseEstimator(
            Constants.Vision.kinematics,
            new Rotation2d(),
            Constants.Vision.lastModulePositions,
            new Pose2d());
  }

  private Pose2d fusedPose = new Pose2d();
  private Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(99999, 99999, 99999);
  private boolean fusedPoseUpdated = false;
  // Check time units
  private double fusedTime = Timer.getFPGATimestamp();
  // Updates fusedPose and fusedStdDevs
  public boolean fuse(Pose2d[] poses, Matrix<N3, N1>[] stdDevs) {
    fusedPoseUpdated = true;
    double possibleTime = Timer.getFPGATimestamp();
    // Shoudl alwyas be flase, but just in case
    if (poses.length != stdDevs.length) {
      Logger.recordOutput("PhotonvisionLogging/Hit length mismatch in fuse", true);
      fusedPoseUpdated = false;
      return false;
    }
    // First check if each pose is in the field
    ArrayList<Pose2d> inFieldPoses = new ArrayList<Pose2d>();
    ArrayList<Matrix> inFieldStdDevs = new ArrayList<Matrix>();
    for (int i = 0; i < poses.length; i++) {
      Pose2d pose = poses[i];
      // X check
      double xDeadspace = 0.05; // For each direction
      if (pose.getX() < 0 - xDeadspace || pose.getX() > 17.5387 + xDeadspace) {
        continue;
      }
      // Y check
      double yDeadspace = 0.05; // For each direction
      if (pose.getY() < 0 - yDeadspace || pose.getY() > 8.0518 + yDeadspace) {
        continue;
      }
      // Now in field, add it to list
      inFieldPoses.add(pose);
      inFieldStdDevs.add(stdDevs[i]);
    }
    if (inFieldPoses.size() == 0) {
      Logger.recordOutput("Poses in field length is 0", true);
      fusedPoseUpdated = false;
      return false;
    }
    if (inFieldPoses.size() == 1) {
      fusedPose = inFieldPoses.get(0);
      fusedStdDevs = inFieldStdDevs.get(0);
      fusedPoseUpdated = true;
      fusedTime = possibleTime;
      return true;
    }
    // Use best pose
    double bestAverage = 999999999;
    Matrix<N3, N1> bestStdDevs = inFieldStdDevs.get(0);
    Pose2d bestPose = inFieldPoses.get(0);
    for (int i = 1; i < inFieldStdDevs.size(); i++) {
      double average =
          inFieldStdDevs.get(i).elementSum()
              / (inFieldStdDevs.get(i).getNumRows() * inFieldStdDevs.get(i).getNumCols());
      if (average < bestAverage) {
        bestAverage = average;
        bestStdDevs = inFieldStdDevs.get(i);
        bestPose = inFieldPoses.get(i);
      }
    }
    fusedPoseUpdated = true;
    fusedPose = bestPose;
    fusedStdDevs = bestStdDevs;
    fusedTime = possibleTime;
    return true;
  }

  public Pose2d getFusedPose() {
    return fusedPose;
  }

  public Matrix<N3, N1> getFusedStdDevs() {
    return fusedStdDevs;
  }

  public double getFusedTime() {
    return fusedTime;
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
          && camera.getLatestStdDevs() != null
          && camera.canSeeTarget()) {
        poses.add(camera.getLatestLocation().toPose2d());
        stdDevs.add(camera.getLatestStdDevs());
      }
    }
    Pose2d[] posesArr = poses.toArray(new Pose2d[0]);
    Matrix[] stdDevsArr = stdDevs.toArray(new Matrix[0]);
    swerveEstimator.update(pigeon.getRotation2d(), drive.getModulePositions());
    Logger.recordOutput("photonvisionLogging/pigeonRot", pigeon.getRotation2d().getDegrees());

    if (fuse(posesArr, stdDevsArr)) {
      swerveEstimator.addVisionMeasurement(
          this.getFusedPose(), this.getFusedTime(), this.getFusedStdDevs().times(2));
      Logger.recordOutput("photonvisionLogging/isUpdatingWCameras", true);
    } else {
      swerveEstimator.addVisionMeasurement(
          this.getFusedPose(), this.getFusedTime(), this.getFusedStdDevs().times(99));
      Logger.recordOutput("photonvisionLogging/isUpdatingWCameras", false);
    }
    // This method will be called once per scheduler run
    Logger.recordOutput("photonvisionLogging/est Pose", getFusedPose());
  }
}
// Ben was here
