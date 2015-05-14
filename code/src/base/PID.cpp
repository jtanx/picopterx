/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * A PID controller is a widely used feedback controller commonly found in
 * industry.
 *
 * This library is a port of Brett Beauregard's Arduino PID library:
 *
 *  http://www.arduino.cc/playground/Code/PIDLibrary
 *
 * The wikipedia article on PID controllers is a good place to start on
 * understanding how they work:
 *
 *  http://en.wikipedia.org/wiki/PID_controller
 *
 * For a clear and elegant explanation of how to implement and tune a
 * controller, the controlguru website by Douglas J. Cooper (who also happened
 * to be Brett's controls professor) is an excellent reference:
 *
 *  http://www.controlguru.com/
 */

/**
 * Includes
 */
#include "PID.h"

using picopter::PID;

PID::PID(float Kc, float tauI, float tauD, float interval) {

    usingFeedForward = false;
    inAuto           = false;

    //Default the limits to the full range of I/O: 3.3V
    //Make sure to set these to more appropriate limits for
    //your application.
    SetInputLimits(0.0, 3.3);
    SetOutputLimits(0.0, 3.3);

    tSample_ = interval;

    SetTunings(Kc, tauI, tauD);

    setPoint_             = 0.0;
    processVariable_      = 0.0;
    prevProcessVariable_  = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;

    accError_ = 0.0;
    bias_     = 0.0;

    realOutput_ = 0.0;

}

void PID::SetInputLimits(float inMin, float inMax) {

    //Make sure we haven't been given impossible values.
    if (inMin >= inMax) {
        return;
    }

    //Rescale the working variables to reflect the changes.
    /*prevProcessVariable_ *= (inMax - inMin) / inSpan_;
    accError_            *= (inMax - inMin) / inSpan_;

    //Make sure the working variables are within the new limits.
    if (prevProcessVariable_ > 1) {
        prevProcessVariable_ = 1;
    } else if (prevProcessVariable_ < 0) {
        prevProcessVariable_ = 0;
    }*/
    
    if (prevProcessVariable_ > inMax) {
        prevProcessVariable_ = inMax;
    } else if (prevProcessVariable_ < inMin) {
        prevProcessVariable_ = inMin;
    }

    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;

}

void PID::SetOutputLimits(float outMin, float outMax) {

    //Make sure we haven't been given impossible values.
    if (outMin >= outMax) {
        return;
    }

    //Rescale the working variables to reflect the changes.
    //prevControllerOutput_ *= (outMax - outMin) / outSpan_;

    //Make sure the working variables are within the new limits.
    if (prevControllerOutput_ > outMax) {
        prevControllerOutput_ = outMax;
    } else if (prevControllerOutput_ < outMin) {
        prevControllerOutput_ = outMin;
    }

    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;

}

void PID::SetTunings(float Kc, float tauI, float tauD) {

    //Verify that the tunings make sense.
    if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
        return;
    }

    //Store raw values to hand back to user on request.
    pParam_ = Kc;
    iParam_ = tauI;
    dParam_ = tauD;

    float tempTauR;

    if (tauI == 0.0) {
        tempTauR = 0.0;
    } else {
        tempTauR = (1.0 / tauI) * tSample_;
    }

    //For "bumpless transfer" we need to rescale the accumulated error.
    if (inAuto) {
        if (tempTauR == 0.0) {
            accError_ = 0.0;
        } else {
            accError_ *= (Kc_ * tauR_) / (Kc * tempTauR);
        }
    }

    Kc_   = Kc;
    tauR_ = tempTauR;
    tauD_ = tauD / tSample_;

}

void PID::Reset(void) {

    float scaledBias = 0.0;

    if (usingFeedForward) {
        //scaledBias = (bias_ - outMin_) / outSpan_;
        scaledBias = (bias_ - outMin_);
    } else {
        //scaledBias = (realOutput_ - outMin_) / outSpan_;
        scaledBias = (realOutput_ - outMin_);
    }

    prevControllerOutput_ = scaledBias;
    //prevProcessVariable_  = (processVariable_ - inMin_) / inSpan_;
    prevProcessVariable_  = (processVariable_ - inMin_);

    //Clear any error in the integral.
    accError_ = 0;

}

void PID::SetMode(int mode) {

    //We were in manual, and we just got set to auto.
    //Reset the controller internals.
    if (mode != 0 && !inAuto) {
        Reset();
    }

    inAuto = (mode != 0);

}

void PID::SetInterval(float interval) {

    if (interval > 0) {
        //Convert the time-based tunings to reflect this change.
        tauR_     *= (interval / tSample_);
        accError_ *= (tSample_ / interval);
        tauD_     *= (interval / tSample_);
        tSample_   = interval;
    }

}

void PID::SetSetPoint(float sp) {
    setPoint_ = sp;
}

void PID::SetProcessValue(float pv) {

    processVariable_ = pv;

}

void PID::SetBias(float bias){

    bias_ = bias;
    usingFeedForward = true;

}

float PID::Compute() {
    float PV = processVariable_;
    if (PV > inMax_) {
        PV = inMax_;
    } else if (PV < inMin_) {
        PV = inMin_;
    }
    
    float SP = setPoint_;
    if (SP > inMax_) {
        SP = inMax_;
    } else if (SP < inMin_) {
        SP = inMin_;
    }
    
    float error = SP - PV;
    
    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= outMax_ && error > 0) && !(prevControllerOutput_ <= outMin_ && error < 0)) {
        accError_ += error;
    }
    
    //Compute the current slope of the input signal.
    float dMeas = (PV - prevProcessVariable_) / tSample_;

    //Perform the PID calculation.
    controllerOutput_ = bias_ + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));

    //Make sure the computed output is within output constraints.
    if (controllerOutput_ < outMin_) {
        controllerOutput_ = outMin_;
    } else if (controllerOutput_ > outMax_) {
        controllerOutput_ = outMax_;
    }

    //Remember this output for the windup check next time.
    prevControllerOutput_ = controllerOutput_;
    //Remember the input for the derivative calculation next time.
    prevProcessVariable_  = PV;

    return controllerOutput_;
}

/*
float PID::Compute() {
    //Pull in the input and setpoint, and scale them into percent span.
    float scaledPV = (processVariable_ - inMin_) / inSpan_;

    if (scaledPV > 1.0) {
        scaledPV = 1.0;
    } else if (scaledPV < 0.0) {
        scaledPV = 0.0;
    }

    float scaledSP = (setPoint_ - inMin_) / inSpan_;
    if (scaledSP > 1.0) {
        scaledSP = 1;
    } else if (scaledSP < 0.0) {
        scaledSP = 0;
    }

    float error = scaledSP - scaledPV;

    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= 0 && error < 0)) {
        accError_ += error;
    }

    //Compute the current slope of the input signal.
    float dMeas = (scaledPV - prevProcessVariable_) / tSample_;

    float scaledBias = 0.0;

    if (usingFeedForward) {
        scaledBias = (bias_ - outMin_) / outSpan_;
    }

    //Perform the PID calculation.
    controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));

    //Make sure the computed output is within output constraints.
    if (controllerOutput_ < 0.0) {
        controllerOutput_ = 0.0;
    } else if (controllerOutput_ > 1.0) {
        controllerOutput_ = 1.0;
    }

    //Remember this output for the windup check next time.
    prevControllerOutput_ = controllerOutput_;
    //Remember the input for the derivative calculation next time.
    prevProcessVariable_  = scaledPV;

    //Scale the output from percent span back out to a real world number.
    return ((controllerOutput_ * outSpan_) + outMin_);

}*/

float PID::GetInMin() {

    return inMin_;

}

float PID::GetInMax() {

    return inMax_;

}

float PID::GetOutMin() {

    return outMin_;

}

float PID::GetOutMax() {

    return outMax_;

}

float PID::GetInterval() {

    return tSample_;

}

float PID::GetPParam() {

    return pParam_;

}

float PID::GetIParam() {

    return iParam_;

}

float PID::GetDParam() {

    return dParam_;

}
