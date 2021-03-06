# FRC-2021-Home-Challenges
This repository contains Team 254's robot code for the 2021 FRC At Home Challenges. The code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions and some of the style conventions used.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download Gradle and needed FRC/Vendor libraries (make sure you have Java 11 or greater, as that is required)
1. Run `./gradlew downloadAll` to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension from the release page on [this repository](https://github.com/wpilibsuite/allwpilib/releases/latest)

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2021-Home-Challenges.ipr` file with IntelliJ
1. When prompted, select import Gradle build

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2021-Home-Challenges` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Contributing
If you want to contribute, please read through and follow our [Git/GitHub Guidelines](https://docs.google.com/document/d/1J28QKWhNm66jvtzVdcwjz8Gw_wemK2HUTICXWQfJ9HU/edit?usp=sharing)
	
## Style Conventions
- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants
- m*** (i.e. `mPathFollower`): Private instance variables