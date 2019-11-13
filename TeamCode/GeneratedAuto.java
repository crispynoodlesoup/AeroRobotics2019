package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GENERATED-Auto", group="generated")
public class GeneratedAuto extends LinearOpMode {
        //28 * 20 / (2ppi * 4.125)
        Double width = 18.0; //inches
        Integer cpr = 383.6; //counts per rotation
        Integer gearratio = 20;
        Double diameter = 3.937;
        Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
        Double bias = 0.8;//default 0.8
        Double meccyBias = 0.9;//change to adjust only strafing movement
        Double conversion = cpi * bias;
        Boolean exit = false;
        ElapsedTime runtime = new ElapsedTime();
        GyroMath gyro = new GyroMath();
        HardwareTestbot robot = new HardwareTestbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);

                waitForStart();
                //
                moveToPosition(25.8, 0.5);
                //
                strafeToPosition(24.4, 0.5);
                //
                moveToPosition(-4.4, 0.5);
                //
                strafeToPosition(-107.4, 0.5);
                //
                moveToPosition(5.4, 0.5);
                //
                moveToPosition(-27, 0.5);
                //
                strafeToPosition(29.6, 0.5);
                //
                moveToPosition(26, 0.5);
                //
                strafeToPosition(19.0, 0.5);
                //
                }
        public void moveToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches*conversion));
                //
                robot.RLeft.setTargetPosition(robot.RLeft.getCurrentPosition() + move);
                robot.FLeft.setTargetPosition(robot.FLeft.getCurrentPosition() + move);
                robot.RRight.setTargetPosition(robot.RRight.getCurrentPosition() + move);
                robot.RLeft.setTargetPosition(robot.RLeft.getCurrentPosition() + move);
                //
                robot.FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                robot.FLeft.setPower(speed);
                robot.RLeft.setPower(speed);
                robot.RLeft.setPower(speed);
                robot.RRight.setPower(speed);
                //
                while (robot.FLeft.isBusy() && robot.RLeft.isBusy() && robot.RLeft.isBusy() && robot.RRight.isBusy()){
                if (exit){
                robot.RLeft.setPower(0);
                robot.FLeft.setPower(0);
                robot.RRight.setPower(0);
                robot.RLeft.setPower(0);
                return;
                }
                }
                robot.RLeft.setPower(0);
                robot.FLeft.setPower(0);
                robot.RRight.setPower(0);
                robot.RLeft.setPower(0);
                return;
                }
        public void turnWithGyro(double degrees,double time){
                runtime.reset();
                while(gyro.getError(degrees)>2 || runtime.seconds() > time){
                        double spin = gyro.calcPID(degrees);
                        double FLPow = spin;
                        double FRPow = spin;
                        double RLPow = spin;
                        double RRPow = spin;
                        // normalize all motor speeds so no values exceeds 100%.
                        FLPow = Range.clip(FLPow, -robot.MAX_POWER, robot.MAX_POWER);
                        FRPow = Range.clip(FRPow, -robot.MAX_POWER, robot.MAX_POWER);
                        RLPow = Range.clip(RLPow, -robot.MAX_POWER, robot.MAX_POWER);
                        RRPow = Range.clip(RRPow, -robot.MAX_POWER, robot.MAX_POWER);
                        // Set drive motor power levels.
                        robot.FLeft.setPower(FLPow);
                        robot.RLeft.setPower(FRPow);
                        robot.RLeft.setPower(RLPow);
                        robot.RRight.setPower(RRPow);
                }
        }
        public void strafeToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches * cpi * meccyBias));
                //
                robot.RLeft.setTargetPosition(robot.RLeft.getCurrentPosition() - move);
                robot.FLeft.setTargetPosition(robot.FLeft.getCurrentPosition() + move);
                robot.RRight.setTargetPosition(robot.RRight.getCurrentPosition() + move);
                robot.RLeft.setTargetPosition(robot.RLeft.getCurrentPosition() - move);
                //
                robot.FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                robot.FLeft.setPower(speed);
                robot.RLeft.setPower(speed);
                robot.RLeft.setPower(speed);
                robot.RRight.setPower(speed);
                //
                while (robot.FLeft.isBusy() && robot.RLeft.isBusy() && robot.RLeft.isBusy() && robot.RRight.isBusy()){}
                robot.RLeft.setPower(0);
                robot.FLeft.setPower(0);
                robot.RRight.setPower(0);
                robot.RLeft.setPower(0);
                return;
                }
        public void turnWithEncoder(double input){
                robot.FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //
                robot.FLeft.setPower(input);
                robot.RLeft.setPower(input);
                robot.RLeft.setPower(-input);
                robot.RRight.setPower(-input);
        }
}
