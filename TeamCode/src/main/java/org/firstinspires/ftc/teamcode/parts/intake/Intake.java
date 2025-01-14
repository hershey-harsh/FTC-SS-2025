package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {
    public int slideTargetPosition;
    public int liftTargetPosition;
    double motorPower = 0;
    private int currentSlidePos;
    private int currentBucketPos;
    public IntakeTasks tasks;
    public boolean isRedGood = true;
    public boolean isYellowGood = true;
    public boolean isBlueGood = false;
    public int lastSample = -1;
    // this is part of the resets lift to 0 each time it hits the limit switch
    private final EdgeConsumer homingVSlideZero = new EdgeConsumer();
    private final EdgeConsumer homingHSlideZero = new EdgeConsumer();
    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0, 0, 0, 0,0 ,0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware) {
        super(parent, "slider", () -> new IntakeControl(0, 0, 0, 0, 0, 0, 0, 0));
        setConfig(settings, hardware);
    }

    private void setSlideToHomeConfig() {
        double power = -0.125;

        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        getHardware().horizSliderMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig() {
        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().horizSliderMotor.setPower(IntakeHardware.slideHoldPower);
        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void slideWithPower(double power, boolean force) {
        if (Math.abs(power) < getSettings().minRegisterVal) return;

        if (power < 0)
            power *= getSettings().maxSlideSpeed;
        else
            power *= getSettings().maxSlideSpeed;

        if (force)
            setSlidePositionUnsafe(getSlidePosition() + (int) power);
        else
            setSlidePosition(getSlidePosition() + (int) power);
    }

//    public void sweepWithPower(double power) {
//        getHardware().intakeFlipperServo.setPower(power);
//    }

    // Some new helper code

    public void setSlidePosition(int position, double power) {
        if (position < getSettings().positionSlideMin || position > getSettings().positionSlideMax) {  // something very wrong so bail
            stopSlide();
            return;
        }
        slideTargetPosition = position;
        stopSlide();   // ???
        getHardware().horizSliderMotor.setTargetPosition(slideTargetPosition);
        getHardware().horizSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setSlidePower(power);
    }
    public void setLiftPosition(int position, double power) {
        if (position < getSettings().positionLiftMin || position > getSettings().positionLiftMax) {  // something very wrong so bail
            stopLift();
            return;
        }
        liftTargetPosition = position;
        stopLift();   // ???
        getHardware().bucketLiftMotor.setTargetPosition(liftTargetPosition);
        getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(power);
    }

    public void stopSlide() { getHardware().horizSliderMotor.setPower(0); }
    public void stopLift() { getHardware().bucketLiftMotor.setPower(0); }
    public void setSlidePower (double m0) { getHardware().horizSliderMotor.setPower(m0); }
    public void setLiftPower (double m1) { getHardware().bucketLiftMotor.setPower(m1); }

    //

    public void setSlidePosition(int position) {
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position) {
        slideTargetPosition = position;
        getHardware().horizSliderMotor.setTargetPosition(position);
    }

    private void changeSlidePosition(int position) {
        if (position > -1) {
            if(position > 0)
                setSlidePosition(getSettings().maxSlidePosition);
            else setSlidePosition(0);
        }
    }

    private void setBucketLiftPosition(int position) {
        if (position == -1) { // go down
            setBucketLiftPositionUnsafe(getSettings().minLiftPosition);
        } else if ( position == 1) { // go up
            setBucketLiftPositionUnsafe(getSettings().bucketMaxPos);
        }
    }

    private void setBucketLiftPositionUnsafe(int position) {
        getHardware().bucketLiftMotor.setTargetPosition(position);
    }

    public boolean isSlideInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public boolean isLiftInTolerance() {
        return Math.abs(liftTargetPosition - currentBucketPos) <= getSettings().tolerance;
    }

    public void setIntakePosition(int position) {
        /* See alternate controls concept in IntakeTeleop */
//        if (position == 2) { // safe
//            stopAllIntakeTasks();
//            tasks.safeTask.restart();
//        } else if ( position == 1) { // go up
//            stopAllIntakeTasks();
//            tasks.prepareToIntakeTask.restart();
//        }else if ( position == 3) { // go up
//            stopAllIntakeTasks();
//            tasks.autoIntakeTask.restart();
//        }else if ( position == 4) { // go up
//            stopAllIntakeTasks();
//            tasks.transferTask.restart();
//        }else if ( position == 5) { // go up
//            stopAllIntakeTasks();
//            tasks.prepareToDepositTask.restart();
//        }else if ( position == 6) { // go up
//            stopAllIntakeTasks();
//            tasks.depositTask.restart();
//        }
    }

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public double getSpecimanClawMax() {
        return getSettings().SpecimanClawMax;
    }
    public double getSpecimanClawMin() {
        return  getSettings().specimanClawMin;
    }
    private void setSpecimanClaw(int position) {
        if (position==-1) {//open specimanServo
            getHardware().pinch.setPosition(getSpecimanClawMax());
        } else if ( position == 1) { // close specimanServo
            getHardware().pinch.setPosition(getSpecimanClawMin());
        }
    }

    public double getIntakeAngleMin() {
        return  getSettings().intakeAngleMin;

    }
    public double getIntakeAngleMax() {
        return getSettings().intakeAngleMax;
    }
    private void setIntakeAngle(int position) {
        if (position==1) {
            getHardware().flipper.setPosition(getIntakeAngleMin());
        }
        else if (position==-1) {
            getHardware().flipper.setPosition(getIntakeAngleMax());
        }
    }

    public void eStop() {
        // stop all tasks in the intake group
        stopAllIntakeTasks();
        // stop the slides
        stopSlide();
        stopLift();
        // stop all the servos
        getHardware().spinner.stop();
        getHardware().flipper.stop();
        getHardware().chute.stop();
        getHardware().pinch.stop();
    }

    public void stopAllIntakeTasks() {
        tasks.movementTask.runCommand(Group.Command.PAUSE);
        tasks.movementTask.getActiveRunnables().clear();    // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    public int getV_Slide_Min() {
        return getSettings().v_Slide_Min;
    }

    private void setV_Slide(int position) {
        if (position == 2) {
            getHardware().bucketLiftMotor.setTargetPosition(50);
        }
        else if (position==-2) {
            getHardware().bucketLiftMotor.setTargetPosition(2800);
        }
        else if (position==-1) {
//            getHardware().v_SlideMotor.setTargetPosition(20);
//            if (getSettings().v_Slide_pos < getV_Slide_Max()) {
//                getSettings().v_Slide_pos = getSettings().v_Slide_pos - 30; // can't modify a setting
//            }
        } else if ( position == 1) {
//            getHardware().v_SlideMotor.setTargetPosition(1440);
//            if (getSettings().v_Slide_pos > getV_Slide_Min()) {
//                getSettings().v_Slide_pos = 1440; // can't modify a setting
//            }
        }
    }

    @Override
    public void onInit() {
        setMotorsToRunConfig();
        tasks = new IntakeTasks(this, parent);
        tasks.constructAutoHome();
        tasks.constructSafeTask();
        tasks.constructPrepareToIntakeTask();
        tasks.constructTransfer();
        tasks.constructPrepareToDepositTask();
        tasks.constructDepositTask();
        tasks.constructAutoIntakeTask();
//        tasks.constructEjectBadSampleTask();
//        tasks.constructPrepareToTransferTask();

        // this is part of the resets lift to 0 each time it hits the limit switch
        homingVSlideZero.setOnRise(() -> {
            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            getHardware().bucketLiftMotor.setTargetPosition(0);
//            liftTargetPosition = 0;
            setLiftPosition(20,0.125);
        });
        //homing hslide slide setup
        homingHSlideZero.setOnRise(() -> {
            getHardware().horizSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setSlidePosition(20,0.125);
        });
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) {
//        sweepWithPower(control.sweeperPower);
//        setSweepPosition(control.sweepLiftPosition);
        changeSlidePosition(control.sweepSlidePosition);
        setBucketLiftPosition(control.bucketLiftPosition);
        setIntakePosition(control.intakePosition);
        //setSpecimanClaw(control.pinchPosition); // jas
        //setV_Slide(control.v_SlidePosition); // jas
        //setIntakeAngle(control.intakeAngleSupplier); // jas

        currentSlidePos = getHardware().horizSliderMotor.getCurrentPosition();
        currentBucketPos = getHardware().bucketLiftMotor.getCurrentPosition();

        homingVSlideZero.accept(getHardware().bucketLiftZeroSwitch.getState());
        homingHSlideZero.accept(getHardware().slideZeroSwitch.getState());
    }

    @Override
    public void onStart() {
        //drive = getBeanManager().getBestMatch(Drive.class, false);
        tasks.startAutoHome();
    }

    @Override
    public void onStop() {
        //drive.removeController(ContollerNames.distanceContoller);
    }

//    public double getDistance() {
//        return ((DistanceSensor) getHardware().colorSensor).getDistance(DistanceUnit.CM);
//    }
//
//    public boolean isSampleGood() {
//        return isSampleGood(getSampleType());
//    }
//    public boolean isSampleGood(int sample) {
//        if (sample==1 && isRedGood) return true;
//        if (sample==2 && isYellowGood) return true;
//        if (sample==3 && isBlueGood) return true;
//        return false;
//    }
//
//    public int getSampleType() {
//        float[] hsvValues = new float[3];
//        NormalizedRGBA colors = getHardware().colorSensor.getNormalizedColors();
//        Color.colorToHSV(colors.toColor(), hsvValues);
//        int hue = (int) hsvValues[0];
//        int type = -1;
//        if (hue < 60) type = 1;                //red     // red sometimes triggers as 60. Better to wait for a better read?
//        if (hue >= 65 && hue <= 160) type = 2; //yellow
//        if (hue > 160) type = 3;               //blue
//        if (hue == 0) type = 0;
//        //lastHue = hue;  // for debugging
//        //lastType = type;
//        lastSample = type;
//        return type;
//    }
}

