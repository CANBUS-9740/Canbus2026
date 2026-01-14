# frc-robot-template

Template for FRC robot projects.

## Migrating to 2026

- WPILib
  - [WPILib 2026.1.1-beta-1 Release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.1.1-beta-1)
  - [WPILib 2026.1.1 Release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.1.1)
- REV
  - [REVLib 2026.0.0-alpha-1 Release](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-alpha-1)
  - [REVLib 2026.0.0-beta-1 Release](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-beta-1)
  - [REVLib 2026.0.0 Release](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0)
  - [REVLib 2026.0.1 Release](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.1)
  - [Hardware Client 2](https://docs.revrobotics.com/rev-hardware-client-2)
- CTRE
  - [Phoenix Changelog](https://api.ctr-electronics.com/changelog)
  - [Phoenix new for 2026](https://v6.docs.ctr-electronics.com/en/latest/docs/yearly-changes/yearly-changelog.html)
  - [Phoenix 2026 Release](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v26.1.0)
- PathPlanner
  - [PathPlanner 2026.1.2 Release](https://github.com/mjansen4857/pathplanner/releases/tag/v2026.1.2)

## Run Configuration

The project has several pre-made run configurations for your convenience. These configurations will be loaded automatically when you open the project.

To use a run configuration, select it from the configurations drop down box and click on _run_.

<img width="298" height="72" alt="image" src="https://github.com/user-attachments/assets/4842c8bc-d891-4222-9376-2a2e5f8deed7" />

### For Real RObot

Select the `Robot Deploy` run configuration and click on run. This will build and deploy the code to the robot.

<img width="256" height="70" alt="image" src="https://github.com/user-attachments/assets/c786f01a-3b08-439c-8e30-726eab720e82" />

Select the `Robot Deploy Debug` run configuration to deploy the code with debugger enabled.

To attach a debugger, after a successful deployment (i.e. wait for the deployment to finish successfully), select and run the `RoboRIO Debugger` (run with debug, the bug button, not the run button).

<img width="286" height="70" alt="image" src="https://github.com/user-attachments/assets/2fb7f2d0-ef48-4306-ac42-5f4e11d87313" />

### For Simulation

Select the `Simulate` run configuration and click on run. This will build and run the code in the simulator.

<img width="235" height="54" alt="image" src="https://github.com/user-attachments/assets/ea6b37c5-77d5-4ce7-be00-7e80d12070c0" />

You can also run with _debugger_ by cliking on the _bug_ button instead of the _run_ button.
