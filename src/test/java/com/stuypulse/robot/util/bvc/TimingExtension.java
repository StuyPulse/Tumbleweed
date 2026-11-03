/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import org.junit.jupiter.api.extension.AfterTestExecutionCallback;
import org.junit.jupiter.api.extension.BeforeTestExecutionCallback;
import org.junit.jupiter.api.extension.ExtensionContext;

public class TimingExtension implements BeforeTestExecutionCallback, AfterTestExecutionCallback {
    private static final String START_TIME = "start_time";

    @Override
    public void beforeTestExecution(ExtensionContext context) {
        getStore(context).put(START_TIME, System.nanoTime());
    }

    @Override
    public void afterTestExecution(ExtensionContext context) {
        long startTime = getStore(context).remove(START_TIME, long.class);
        long duration = System.nanoTime() - startTime;

        System.out.println(
                "Method "
                        + context.getRequiredTestMethod().getName()
                        + " took "
                        + (duration / 1e6)
                        + " ms");
    }

    private ExtensionContext.Store getStore(ExtensionContext context) {
        return context.getStore(
                ExtensionContext.Namespace.create(getClass(), context.getRequiredTestMethod()));
    }
}
