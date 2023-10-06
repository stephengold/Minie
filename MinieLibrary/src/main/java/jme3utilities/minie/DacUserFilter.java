/*
 Copyright (c) 2020, Stephen Gold
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

import com.jme3.bullet.animation.DacLinks;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import java.util.logging.Logger;

/**
 * A simple DebugAppStateFilter that selects physics objects associated with a
 * specific DynamicAnimControl. Instances are immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DacUserFilter implements BulletDebugAppState.DebugAppStateFilter {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(DacUserFilter.class.getName());
    // *************************************************************************
    // fields

    /**
     * DynamicAnimControl (may be null)
     */
    final private DacLinks dac;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a filter for the specified DynamicAnimControl.
     *
     * @param dac the desired DynamicAnimControl, or null to display/dump
     * objects with no DynamicAnimControl
     */
    public DacUserFilter(DacLinks dac) {
        this.dac = dac;
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed/dumped.
     * Note: recursive!
     *
     * @param physicsObject the joint or collision object to test (unaffected)
     * @return return true if the object should be displayed/dumped, false if it
     * shouldn't be
     */
    @Override
    public boolean displayObject(Object physicsObject) {
        boolean result = false;

        if (physicsObject instanceof PhysicsCollisionObject) {
            PhysicsCollisionObject pco = (PhysicsCollisionObject) physicsObject;
            Object user = pco.getUserObject();
            if (user instanceof PhysicsLink) {
                PhysicsLink link = (PhysicsLink) user;
                if (link.getControl() == dac) {
                    result = true;
                }
            }

        } else if (physicsObject instanceof PhysicsJoint) {
            PhysicsJoint joint = (PhysicsJoint) physicsObject;
            PhysicsBody a = joint.getBody(JointEnd.A);
            if (displayObject(a)) {
                result = true;
            } else {
                PhysicsBody b = joint.getBody(JointEnd.B);
                if (displayObject(b)) {
                    result = true;
                }
            }
        }

        return result;
    }
}
