/*
 Copyright (c) 2023, Stephen Gold
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
import java.util.logging.Logger;

/**
 * A simple DebugAppStateFilter to select objects that are assignable to a
 * specific class. Instances are immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ClassFilter implements BulletDebugAppState.DebugAppStateFilter {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(ClassFilter.class.getName());
    // *************************************************************************
    // fields

    /**
     * class to select (may be null)
     */
    final private Class<?> clazz;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a filter for the specified class.
     *
     * @param clazz the class to select
     */
    public ClassFilter(Class<?> clazz) {
        this.clazz = clazz;
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed/dumped.
     *
     * @param physicsObject the joint or collision object to test (unaffected)
     * @return return true if physicsObject should be displayed/dumped, false if
     * it shouldn't be
     */
    @Override
    public boolean displayObject(Object physicsObject) {
        if (clazz.isAssignableFrom(physicsObject.getClass())) {
            return true;
        } else {
            return false;
        }
    }
}
