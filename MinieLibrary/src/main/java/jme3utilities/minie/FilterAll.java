/*
 Copyright (c) 2019, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie;

import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.export.Savable;
import java.util.logging.Logger;

/**
 * A very simple DebugAppStateFilter that always returns the same value.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class FilterAll implements BulletDebugAppState.DebugAppStateFilter {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(FilterAll.class.getName());
    // *************************************************************************
    // fields

    /**
     * value returned by displayObject()
     */
    final private boolean returnValue;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new filter.
     *
     * @param returnValue value to be returned by displayObject()
     */
    public FilterAll(boolean returnValue) {
        this.returnValue = returnValue;
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed.
     *
     * @param physicsObject the joint or collision object to test (unaffected)
     * @return return true if the object should be displayed, false if not
     */
    @Override
    public boolean displayObject(Savable physicsObject) {
        return returnValue;
    }
}
