
/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAuto", group="StarterBot")
//@Disabled
public class StarterBotAuto extends OpMode
{
    /*
     * SHOOTER SETTINGS
     *
     * Starting speed: 3.5 RPM
     * D-pad Up: +1 RPM
     * D-pad Down: -1 RPM
     *
     * The launcher encoder has 28 ticks per revolution.
     */

    public double launcherTargetRPM = 3.5;

    public final double LAUNCHER_RPM_INCREMENT = 1.0;
    public final double LAUNCHER_ENCODER_TICKS_PER_REV = 28.0;

    // Tracks whether the D-pad buttons were pressed during the previous loop.
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    /*
     * DRIVETRAIN SETTINGS
     */

    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM =
            (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;

    double intakePower = 0;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;

    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    private AutonomousState autonomousState;

    /*
     * INITIALIZATION
     */

    @Override
    public void init() {

        autonomousState = AutonomousState.LAUNCH;

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter Starting RPM", launcherTargetRPM);
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    /*
     * MAIN AUTONOMOUS LOOP
     */

    @Override
    public void loop() {

        switch (autonomousState) {

            case LAUNCH:

                // Read PS5 D-pad input and update the shooter's target RPM.
                updateShooterRPM();

                launch(true);

                if (autoTimer.seconds() > 10) {
                    launch(false);
                    intakePower = 0;
                    autonomousState = AutonomousState.DRIVE;
                }

                break;

            case DRIVE:

                if (drive(DRIVE_SPEED, -120, DistanceUnit.MM, 2)) {
                    autonomousState = AutonomousState.COMPLETE;
                }

                break;

            case COMPLETE:

                telemetry.addLine("Auto Complete!");

                break;
        }

        intake.setPower(intakePower);

        telemetry.addData("AutoState", autonomousState);

        telemetry.addData(
                "Motor Current Positions",
                "left (%d), right (%d)",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition()
        );

        telemetry.addData(
                "Motor Target Positions",
                "left (%d), right (%d)",
                leftDrive.getTargetPosition(),
                rightDrive.getTargetPosition()
        );

        telemetry.addData("Shooter Target RPM", launcherTargetRPM);

        telemetry.addData(
                "Shooter Actual RPM",
                launcher.getVelocity() * 60.0 /
                        LAUNCHER_ENCODER_TICKS_PER_REV
        );

        telemetry.update();
    }

    @Override
    public void stop() {
    }

    /*
     * SHOOTER RPM CONTROL
     *
     * D-pad Up increases RPM by 1.
     * D-pad Down decreases RPM by 1.
     *
     * Only registers a new press after the button has been released.
     */

    void updateShooterRPM() {

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !previousDpadUp) {
            launcherTargetRPM += LAUNCHER_RPM_INCREMENT;
        }

        if (dpadDown && !previousDpadDown) {
            launcherTargetRPM -= LAUNCHER_RPM_INCREMENT;
        }

        // Prevent the target RPM from going below zero.
        launcherTargetRPM = Math.max(0, launcherTargetRPM);

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
    }

    /*
     * LAUNCHER CONTROL
     *
     * Converts the selected RPM into encoder ticks per second.
     */

    void launch(boolean input) {

        double targetVelocity =
                (launcherTargetRPM * LAUNCHER_ENCODER_TICKS_PER_REV) / 60.0;

        if (input) {

            launcher.setVelocity(targetVelocity);

        } else {

            launcher.setVelocity(0);
        }

        // Run the windmill when the launcher reaches its selected speed.
        if (input && targetVelocity > 0 &&
                launcher.getVelocity() >= targetVelocity * 0.9) {

            windmillServo.setPower(1);
            intakePower += 0.5;

        } else {

            windmillServo.setPower(0);
        }
    }

    /*
     * DRIVE FUNCTION
     */

    boolean drive(
            double speed,
            double distance,
            DistanceUnit distanceUnit,
            double holdSeconds
    ) {

        final double TOLERANCE_MM = 10;

        double targetPosition =
                (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if (Math.abs(targetPosition - leftDrive.getCurrentPosition()) >
                (TOLERANCE_MM * TICKS_PER_MM)) {

            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /*
     * ROTATE FUNCTION
     */

    boolean rotate(
            double speed,
            double angle,
            AngleUnit angleUnit,
            double holdSeconds
    ) {

        final double TOLERANCE_MM = 10;

        double targetMm =
                angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if (Math.abs(leftTargetPosition - leftDrive.getCurrentPosition()) >
                (TOLERANCE_MM * TICKS_PER_MM)) {

            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}
