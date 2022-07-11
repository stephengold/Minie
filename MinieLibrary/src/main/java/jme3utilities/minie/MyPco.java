/*
 Copyright (c) 2017-2022, Stephen Gold
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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Utility methods that operate on physics collision objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyPco {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger = Logger.getLogger(MyPco.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyPco() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Briefly describe a rigid body for MyControlP or PhysicsDumper. TODO add a
     * similar method for PhysicsCharacter
     *
     * @param body (not null, unaffected)
     * @return a descriptive string (not null, not empty)
     */
    public static String describe(PhysicsRigidBody body) {
        String result;
        if (body.isStatic()) {
            result = "Sta";
        } else if (body.isKinematic()) {
            result = "Kin";
        } else {
            float mass = body.getMass();
            String massText = MyString.describe(mass);
            String activeText = body.isActive() ? "" : "/inactive";
            result = String.format("Dyn(mass=%s)%s", massText, activeText);
        }

        if (!body.isContactResponse()) {
            result += "/NOresponse";
        }
        if (!body.isInWorld()) {
            result += "/NOspace";
        }

        return result;
    }

    /**
     * Generate a name for the specified collision object.
     *
     * @param pco object to name (not null, unaffected)
     * @return the name (not null, not empty)
     */
    public static String objectName(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "physics object");

        long id = pco.nativeId();
        String name;
        if (pco instanceof PhysicsCharacter) {
            name = String.format("chara%d", id);
        } else if (pco instanceof PhysicsGhostObject) {
            name = String.format("ghost%d", id);
        } else if (pco instanceof PhysicsVehicle) {
            // must test before RigidBody
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

    /**
     * Parse the ID of a collision object from its name.
     *
     * @param name the input text (not null, length&gt;5)
     * @return the object's ID
     *
     * @see #objectName(com.jme3.bullet.collision.PhysicsCollisionObject)
     */
    public static long parseId(String name) {
        Validate.nonEmpty(name, "name");

        if (name.length() <= 5) {
            throw new IllegalArgumentException("name=" + MyString.quote(name));
        }
        String decimal = name.substring(5);
        long result = Long.parseLong(decimal);

        return result;
    }
}
