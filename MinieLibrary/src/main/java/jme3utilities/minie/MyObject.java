/*
 Copyright (c) 2017-2018, Stephen Gold
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

import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Utility methods that operate on physics collision objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyObject.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyObject() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Briefly describe a rigid body for MyControlP or PhysicsDumper.
     *
     * @param body (not null, unaffected)
     * @return a descriptive string (not null, not empty)
     */
    public static String describe(PhysicsRigidBody body) {
        String result;
        float mass = body.getMass();
        if (mass == RigidBodyControl.massForStatic) {
            result = "sta";
        } else if (body.isKinematic()) {
            result = "kin";
        } else {
            boolean active = body.isActive();
            result = String.format("dyn/%f kg/%sactive", mass,
                    active ? "" : "in");
        }

        return result;
    }

    /**
     * Describe the user of a collision object.
     *
     * @param pco the collision object to describe (not null, unaffected)
     * @return a descriptive string (not null)
     */
    public static String describeUser(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        StringBuilder builder = new StringBuilder(32);
        Object user = pco.getUserObject();
        if (user != null) {
            builder.append(" user=");
            builder.append(user.getClass().getSimpleName());
            if (user instanceof Spatial) {
                Spatial spatial = (Spatial) user;
                String name = spatial.getName();
                String text = MyString.quote(name);
                builder.append(text);
            } else if (user instanceof PhysicsLink) {
                PhysicsLink link = (PhysicsLink) user;
                String name = link.boneName();
                String text = MyString.quote(name);
                builder.append(text);
            }
        }

        return builder.toString();
    }

    /**
     * Generate a name for the specified physics object.
     *
     * @param pco object to name (not null, unaffected)
     * @return the name (not null, not empty)
     */
    public static String objectName(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "physics object");

        long id = pco.getObjectId();
        String name;
        if (pco instanceof PhysicsCharacter) {
            name = String.format("chara%d", id);
        } else if (pco instanceof PhysicsGhostObject) {
            name = String.format("ghost%d", id);
        } else if (pco instanceof PhysicsVehicle) {// must test before RigidBody
            name = String.format("vehic%d", id);
        } else if (pco instanceof PhysicsRigidBody) {
            name = String.format("rigid%d", id);
        } else {
            String typeName = pco.getClass().getCanonicalName();
            String msg = "Unknown type of collision object: " + typeName;
            throw new IllegalArgumentException(msg);
        }

        return name;
    }
}
