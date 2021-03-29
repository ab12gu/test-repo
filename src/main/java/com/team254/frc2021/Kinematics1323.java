package com.team254.frc2021;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.frc2021.subsystems.Drive;
import com.team254.frc2021.subsystems.SwerveModule;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kinematics1323 {

    private static final int kNumberOfModules = 4;
    private static List<Translation2d> moduleRelativePositions = Constants.kModulePositions;
	private static List<Translation2d> moduleRotationDirections = updateRotationDirections();
			
	private static List<Translation2d> updateRotationDirections(){
		List<Translation2d> directions = new ArrayList<>(kNumberOfModules);
		for(int i = 0; i < kNumberOfModules; i++){
			directions.add(moduleRelativePositions.get(i).rotateBy(Rotation2d.fromDegrees(90)));
		}
		return directions;
	}

    public static DriveSignal inverseKinematics(Translation2d translationalVector, double rotationalMagnitude, boolean field_relative) {
        List<Translation2d> driveVectors = new ArrayList<>(kNumberOfModules);

		if(field_relative) {
			translationalVector = translationalVector.rotateBy(Drive.getInstance().getHeading());
		}

		for(int i = 0; i < kNumberOfModules; i++){
			driveVectors.add(translationalVector.translateBy(moduleRotationDirections.get(i).scale(rotationalMagnitude)));
		}
		double maxMagnitude = 1.0;
		for(Translation2d t : driveVectors){
			double magnitude = t.norm();
			if(magnitude > maxMagnitude){
				maxMagnitude = magnitude;
			}
		}
		for(int i = 0; i < kNumberOfModules; i++){
			Translation2d driveVector = driveVectors.get(i);
			driveVectors.set(i, driveVector.scale(1.0/maxMagnitude));
		}

        double [] wheelSpeeds = new double[kNumberOfModules];
        Rotation2d [] wheelAzimuths = new Rotation2d[kNumberOfModules];
        // System.out.println()
        for (int i = 0; i < kNumberOfModules; i++) {
            wheelSpeeds[i] = driveVectors.get(i).norm();
            wheelAzimuths[i] = driveVectors.get(i).direction();
        }
		return new DriveSignal(wheelSpeeds, wheelAzimuths);
    }


//    public static DriveSignal forwardKinematics(DriveSignal driveSignal, Rotation2d gyroHeading, Pose2d currentPose) {
//		double x = 0.0;
//		double y = 0.0;
//
//		double[][] distances = new double[4][2];
//
//		Rotation2d[] moduleAngles = new Rotation2d[4];
//		for(int i = 0; i < 4; i++){
//			moduleAngles[i] = driveSignal.getWheelAzimuths()[i].rotateBy(gyroHeading);
//			double distance = m.getEstimatedRobotPose().getTranslation().distance(currentPose.getTranslation());
//			distances[m.moduleID][0] = m.moduleID;
//			distances[m.moduleID][1] = distance;
//		}
//
//		Arrays.sort(distances, new java.util.Comparator<double[]>() {
//			public int compare(double[] a, double[] b) {
//				return Double.compare(a[1], b[1]);
//			}
//		});
//		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
//		double firstDifference = distances[1][1] - distances[0][1];
//		double secondDifference = distances[2][1] - distances[1][1];
//		double thirdDifference = distances[3][1] - distances[2][1];
//		if(secondDifference > (1.5 * firstDifference)){
//			modulesToUse.add(modules.get((int)distances[0][0]));
//			modulesToUse.add(modules.get((int)distances[1][0]));
//		}else if(thirdDifference > (1.5 * firstDifference)){
//			modulesToUse.add(modules.get((int)distances[0][0]));
//			modulesToUse.add(modules.get((int)distances[1][0]));
//			modulesToUse.add(modules.get((int)distances[2][0]));
//		}else{
//			modulesToUse.add(modules.get((int)distances[0][0]));
//			modulesToUse.add(modules.get((int)distances[1][0]));
//			modulesToUse.add(modules.get((int)distances[2][0]));
//			modulesToUse.add(modules.get((int)distances[3][0]));
//		}
//
//		SmartDashboard.putNumber("Modules Used", modulesToUse.size());
//
//		for(SwerveDriveModule m : modulesToUse){
//			x += m.getEstimatedRobotPose().getTranslation().x();
//			y += m.getEstimatedRobotPose().getTranslation().y();
//		}
//
//		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
//		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
//		distanceTraveled += deltaPos;
//		pose = updatedPose;
//		modules.forEach((m) -> m.resetPose(pose));
//	}
    
}
