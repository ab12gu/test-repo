package com.team254.frc2021;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

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

    public static DriveSignal inverseKinematics(Translation2d translationalVector, double rotationalMagnitude) {
        List<Translation2d> driveVectors = new ArrayList<>(kNumberOfModules);
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


    
}
