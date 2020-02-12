package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="RedCamAuto", group="generated")
public class CamAuto_Red extends LinearOpMode {
        //width = 18.0; //inches
        //cpr = 383.6; //counts per rotation
        //gearratio = 13.7;
        //diameter = 3.937;
        //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
        //bias = 0.205;
        int conversion = 88;
        
        // meccyBias = 0.21;//change to adjust only strafing movement
        int straifConv = 89;
        
        int colorOffset  = 0;
        
        int r; 
        
        Boolean exit = false;
        double redSide = -1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AZjoEnH/////AAABmXdFD2Xsrk4krInf+EdRY0NaRrdzvbptLaUoVN2kuF2/FnuWVscRF9ozak4bIpJCr1SLehfzrXHS+H3Z7XMNIgxwg6lttQ4zp7ODEDt1XQ/DLQcjpmYXruF4eBBRsIBey35Ue6g4E51WOebmNW/aDFDhz3zON+NNYbyk/4XOszsw7CwHpcNLBXqT0prM/NYwkCaJFocA8cpWcViM0Mka8kEV+T1X1ZtRnPwMxtQrxO19ksdbRv0bjPmco0iiOAvRwMcyVxg250tckD64iSWJkIhlqakYMLA1r00YPtUY4VSfShG0pWTDn/RF9/TqhM8qICp9ZPCz5QlPn8qt4cfiofTjzE41R+VvjKnIGK1B9g5o";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private int detect = -1;
    private int path = -1;
    private int checkCnt = 0;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        
        //robot.liftPos(17000);
        //sleep(500);
        robot.servoGrab1.setPosition(1);
        robot.servoGrab2.setPosition(0.6);
        robot.servoArm.setPosition(0.4);
        initializeAll();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while (detect == -1) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    checkCnt ++;
                    if(checkCnt > 5) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) { calcRoll(recognition.getLeft()); }
                            telemetry.addData("path", path);
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) { tfod.shutdown();}
        waitForStart();
        if(path == 0) {
            runRight();
        } else if(path == 1) {
            runMiddle();
        } else {
            runLeft();
        }
    }
    private void calcRoll(double leftX){
        if(leftX < 60)
            path = 2;
        else if(leftX >= 100 && leftX < 250)
            path = 1;
        else
            path = 0;
        detect = 0;
    }
    private void runLeft(){
        telemetry.addData("Running Path: ", path);
        telemetry.update();
                strafeToPosition(-31.5, 1, 0);
                //
                moveToPosition(-8*redSide, 1, 0);
                //
                strafeToPosition(-1.2, 1, 0);
                //
                robot.servoArm.setPosition(1);
                //
                sleep(400);
                //
                strafeToPosition(10, 1, 0);
                //
                moveToPosition(38.5*redSide, 1, 0);
                //
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-77.0*redSide, 1, 0);
                //
                strafeToPosition(-12, 1, 0);
                //
                robot.servoArm.setPosition(0.95);
                //
                sleep(400);
                //
                strafeToPosition(12.0, 1, 0);
                //
                moveToPosition(44.0*redSide, 1, 0);
                //uwu
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-16.0*redSide, 1, 0);
                //
                strafeToPosition(12.0, 1, 0);
                
    }
    private void runMiddle(){
        telemetry.addData("Running Path: ", path);
        telemetry.update();
                strafeToPosition(-31.5, 1, 0);
                //
                strafeToPosition(-1.2, 1, 0);
                //
                robot.servoArm.setPosition(1);
                //
                sleep(400);
                //
                strafeToPosition(10, 1, 0);
                //
                moveToPosition(38.5*redSide, 1, 0);
                //
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-77.0*redSide, 1, 0);
                //
                strafeToPosition(-12, 1, 0);
                //
                robot.servoArm.setPosition(0.95);
                //
                sleep(400);
                //
                strafeToPosition(12.0, 1, 0);
                //
                moveToPosition(44.0*redSide, 1, 0);
                //uwu
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-16.0*redSide, 1, 0);
                //
                strafeToPosition(12.0, 1, 0);
                
    }
    private void runRight(){
        telemetry.addData("Running Path: ", path);
        telemetry.update();
                strafeToPosition(-31.5, 1, 0);
                //
                moveToPosition(8*redSide, 1, 0);
                //
                strafeToPosition(-1.2, 1, 0);
                //
                robot.servoArm.setPosition(1);
                //
                sleep(400);
                //
                strafeToPosition(10, 1, 0);
                //
                moveToPosition(38.5*redSide, 1, 0);
                //
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-77.0*redSide, 1, 0);
                //
                strafeToPosition(-12, 1, 0);
                //
                robot.servoArm.setPosition(0.95);
                //
                sleep(400);
                //
                strafeToPosition(12.0, 1, 0);
                //
                moveToPosition(44.0*redSide, 1, 0);
                //uwu
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-16.0*redSide, 1, 0);
                //
                strafeToPosition(12.0, 1, 0);
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void initializeAll(){
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
    }
        public void moveToPosition(double inches, double speed, double target){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 18/26));
                decelInches = (int)Range.clip(decelInches, 0, 1800);
                double speedTemp = speed;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 800) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                speed = Range.clip(speed, 0.1, 1.0);
                driveStraight(target,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 800) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1800.0;
                        speed = Range.clip(speed, 0.1, 1.0);
                        driveStraight(target,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void strafeToPosition(double inches, double speed, double target){
                //
                int move = (int)(Math.round(inches * straifConv));
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() - move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                speed = Range.clip(speed, 0.1, 1);
                straifStraight(target, speed, r);
                //
                while (robot.leftRear.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.rightRear.isBusy()){
                        if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                        if(Math.abs(end - robot.leftFront.getCurrentPosition()) < 1500) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1600.0;
                        speed = Range.clip(speed, 0.1, 1);
                        straifStraight(target, speed, r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void turnToTarget(double target) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                for(int i = 0; i < 5; i++) {
                        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                        while (gyro.calcPID(target) != 0 && !isStopRequested()){
                                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                                telemetry.update();
                        }
                        sleep(20);
                }
        }
        public void liftPos(int pos) {
                robot.lift.setTargetPosition(pos);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                while(robot.lift.isBusy()){}
        }
        
        public void driveArc(double target, double speed, double turnSpeed) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                while(Math.abs(gyro.calcPID(target)) > 0.2) {
                        robot.moveLateral(speed, turnSpeed*2, 0, 0);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                        telemetry.update();                
                }
                turnToTarget(target);
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, gyro.calcPID(target)*reverse, speed, 0);
        }
}
