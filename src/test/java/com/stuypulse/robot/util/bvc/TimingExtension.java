/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import jdk.jfr.Category;
import jdk.jfr.Event;
import jdk.jfr.Label;
import jdk.jfr.Name;
import jdk.jfr.StackTrace;
import org.junit.jupiter.api.extension.AfterTestExecutionCallback;
import org.junit.jupiter.api.extension.BeforeTestExecutionCallback;
import org.junit.jupiter.api.extension.ExtensionContext;

public class TimingExtension implements BeforeTestExecutionCallback, AfterTestExecutionCallback {
    @Name("junit.TestMethod")
    @Label("Test Method")
    @Category("Tests")
    @StackTrace(true)
    static class TestMethodEvent extends Event {
        @Label("Class")
        public String className;

        @Label("Method")
        public String methodName;
    }

    @Override
    public void beforeTestExecution(ExtensionContext ctx) {
        TestMethodEvent e = new TestMethodEvent();
        e.className = ctx.getRequiredTestClass().getName();
        e.methodName = ctx.getRequiredTestMethod().getName();
        e.begin();
        getStore(ctx).put("jfr_event", e);
    }

    @Override
    public void afterTestExecution(ExtensionContext ctx) {
        getStore(ctx).remove("jfr_event", TestMethodEvent.class).commit();
    }

    private ExtensionContext.Store getStore(ExtensionContext context) {
        return context.getStore(
                ExtensionContext.Namespace.create(getClass(), context.getRequiredTestMethod()));
    }
}
