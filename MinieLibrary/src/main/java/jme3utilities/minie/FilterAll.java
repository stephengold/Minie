/*
 Copyright (c) 2019-2024 Stephen Gold
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
import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A simple DebugAppStateFilter that returns true for most physics objects (or
 * false for most physics objects) with specified exceptions.
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
     * value returned by displayObject() for non-exceptional arguments
     */
    final private boolean returnValue;
    /**
     * physics objects that are exceptions
     */
    final private Collection<Object> exceptions = new ArrayList<>(32);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a filter with the specified default return value.
     *
     * @param returnValue the default value to be returned by displayObject()
     * for non-exceptional arguments
     */
    public FilterAll(boolean returnValue) {
        this.returnValue = returnValue;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add an exceptional object to the list.
     *
     * @param exception the object to add (not null, alias created)
     */
    public void addException(Object exception) {
        Validate.nonNull(exception, "exception");

        if (!exceptions.contains(exception)) {
            exceptions.add(exception);
        }
    }

    /**
     * Clear the list of exceptions.
     */
    public void clearExceptions() {
        exceptions.clear();
    }

    /**
     * Count how many exceptions there are.
     *
     * @return the count (&ge;0)
     */
    public int countExceptions() {
        int result = exceptions.size();
        return result;
    }

    /**
     * Return the value returned by displayObject() for non-exceptional
     * arguments.
     *
     * @return the boolean value
     */
    public boolean defaultReturnValue() {
        return returnValue;
    }

    /**
     * Enumerate the exceptions.
     *
     * @return a new array of pre-existing objects (not null)
     */
    public Object[] listExceptions() {
        int numExceptions = countExceptions();
        Object[] result = new Object[numExceptions];
        exceptions.toArray(result);

        return result;
    }

    /**
     * Remove an object from the exceptions list.
     *
     * @param exception the object to remove (not null, unaffected)
     */
    public void removeException(Object exception) {
        Validate.nonNull(exception, "exception");

        boolean success = exceptions.remove(exception);
        assert success;
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed/dumped.
     *
     * @param physicsObject the joint or collision object to test (unaffected)
     * @return return true if the object should be displayed/dumped, false if
     * not
     */
    @Override
    public boolean displayObject(Object physicsObject) {
        if (exceptions.contains(physicsObject)) {
            return !returnValue;
        } else {
            return returnValue;
        }
    }
}
