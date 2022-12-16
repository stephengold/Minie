/*
 Copyright (c) 2013-2022, Stephen Gold
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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.control.AbstractPhysicsControl;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.control.VehicleControl;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyControl;
import jme3utilities.Validate;

/**
 * Utility methods that operate on scene-graph controls/spatials/subtrees that
 * may include physics controls.
 *
 * @see jme3utilities.MyControl
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyControlP {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyControlP.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyControlP() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Check whether a scene-graph control implements applyPhysicsLocal().
     *
     * @param sgc the control to test (may be null, unaffected)
     * @return true if it's implemented, otherwise false
     */
    public static boolean canApplyPhysicsLocal(Control sgc) {
        boolean result = sgc instanceof AbstractPhysicsControl
                || sgc instanceof GhostControl
                || sgc instanceof RigidBodyControl
                || sgc instanceof VehicleControl;
        if (sgc instanceof DynamicAnimControl) {
            result = false;
        }

        return result;
    }

    /**
     * Check whether a scene-graph control implements isEnabled() and
     * setEnabled().
     *
     * @param sgc control to test (may be null, unaffected)
     * @return true if it's implemented, otherwise false
     */
    public static boolean canDisable(Control sgc) {
        boolean result = MyControl.canDisable(sgc)
                || sgc instanceof PhysicsControl;

        return result;
    }

    /**
     * Generate a textual description of a scene-graph control.
     *
     * @param sgc instance to describe (not null, unaffected)
     * @return description (not null, not empty)
     */
    public static String describe(Control sgc) {
        String result;

        if (sgc instanceof RigidBodyControl) {
            StringBuilder builder = new StringBuilder(60);

            String type = MyControl.describeType(sgc);
            builder.append(type);

            RigidBodyControl rigidBodyControl = (RigidBodyControl) sgc;
            builder.append('[');

            String desc = MyPco.describe(rigidBodyControl);
            builder.append(desc);

            builder.append(' ');

            if (!rigidBodyControl.isInWorld()) {
                builder.append("NOT");
            }
            builder.append("inWorld,");

            if (!rigidBodyControl.isActive()) {
                builder.append("NOT");
            }
            builder.append("active,");

            if (!rigidBodyControl.isApplyScale()) {
                builder.append("NOT");
            }
            builder.append("applyScale,");

            if (!rigidBodyControl.isApplyPhysicsLocal()) {
                builder.append("NOT");
            }
            builder.append("applyLocal]");

            result = builder.toString();

        } else if (sgc instanceof DynamicAnimControl) {
            StringBuilder builder = new StringBuilder(60);

            String type = MyControl.describeType(sgc);
            builder.append(type);

            DynamicAnimControl dac = (DynamicAnimControl) sgc;
            builder.append('[');
            int numLinks = dac.countLinks();
            builder.append(numLinks);
            builder.append(']');

            result = builder.toString();

        } else if (sgc instanceof GhostControl) {
            StringBuilder builder = new StringBuilder(60);

            String type = MyControl.describeType(sgc);
            builder.append(type);

            GhostControl ghostControl = (GhostControl) sgc;
            builder.append('[');

            if (!ghostControl.isApplyScale()) {
                builder.append("NOT");
            }
            builder.append("applyScale,");

            if (!ghostControl.isApplyPhysicsLocal()) {
                builder.append("NOT");
            }
            builder.append("applyLocal]");

            result = builder.toString();

        } else {
            result = MyControl.describe(sgc);
        }

        return result;
    }

    /**
     * Disable all physics controls added to the specified subtree of the scene
     * graph. Disabling these controls removes any collision objects they may
     * have added to physics spaces. Note: recursive!
     *
     * @param subtree (not null)
     */
    public static void disablePhysicsControls(Spatial subtree) {
        int numControls = subtree.getNumControls();
        for (int controlI = 0; controlI < numControls; ++controlI) {
            Control control = subtree.getControl(controlI);
            if (control instanceof PhysicsControl) {
                setEnabled(control, false);
            }
        }
        if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                disablePhysicsControls(child);
            }
        }
    }

    /**
     * Enable all physics controls added to the specified subtree of the scene
     * graph and configure their physics spaces. Note: recursive!
     *
     * @param subtree (not null)
     * @param space the PhysicsSpace to add to, or null for none
     */
    public static void enablePhysicsControls(
            Spatial subtree, PhysicsSpace space) {
        int numControls = subtree.getNumControls();
        for (int controlI = 0; controlI < numControls; ++controlI) {
            Control control = subtree.getControl(controlI);
            if (control instanceof PhysicsControl) {
                PhysicsControl pc = (PhysicsControl) control;
                pc.setPhysicsSpace(space);
                pc.setEnabled(true);
            }
        }
        if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                enablePhysicsControls(child, space);
            }
        }
    }

    /**
     * Access the first enabled RigidBodyControl added to a Spatial.
     *
     * @param spatial spatial to search (not null, unaffected)
     * @return the pre-existing control, or null if none found
     */
    public static RigidBodyControl findEnabledRbc(Spatial spatial) {
        RigidBodyControl result = null;
        int numControls = spatial.getNumControls();
        for (int controlI = 0; controlI < numControls; ++controlI) {
            Control control = spatial.getControl(controlI);
            if (control instanceof RigidBodyControl) {
                RigidBodyControl rbc = (RigidBodyControl) control;
                if (rbc.isEnabled()) {
                    result = rbc;
                    break;
                }
            }
        }

        return result;
    }

    /**
     * Test whether the specified SGC applies physics coordinates to its
     * spatial's local translation.
     *
     * @param sgc which scene-graph control (may be null, unaffected)
     * @return true if applied to local translation, otherwise false
     */
    public static boolean isApplyPhysicsLocal(Control sgc) {
        Validate.nonNull(sgc, "control");

        boolean result;
        if (sgc instanceof AbstractPhysicsControl) {
            AbstractPhysicsControl apc = (AbstractPhysicsControl) sgc;
            result = apc.isApplyPhysicsLocal();

        } else if (sgc instanceof GhostControl) {
            GhostControl gc = (GhostControl) sgc;
            result = gc.isApplyPhysicsLocal();

        } else if (sgc instanceof RigidBodyControl) {
            RigidBodyControl rbc = (RigidBodyControl) sgc;
            result = rbc.isApplyPhysicsLocal();

        } else if (sgc instanceof VehicleControl) {
            VehicleControl vc = (VehicleControl) sgc;
            result = vc.isApplyPhysicsLocal();

        } else {
            String typeName = sgc.getClass().getCanonicalName();
            String message = typeName + " does not support local physics.";
            throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Test whether a scene-graph control is enabled.
     *
     * @param sgc control to test (not null, unaffected)
     * @return true if the control is enabled, otherwise false
     */
    public static boolean isEnabled(Control sgc) {
        Validate.nonNull(sgc, "control");

        boolean result;
        if (sgc instanceof PhysicsControl) {
            PhysicsControl physicsControl = (PhysicsControl) sgc;
            result = physicsControl.isEnabled();
        } else {
            result = MyControl.isEnabled(sgc);
        }

        return result;
    }

    /**
     * Test whether a Spatial is physics-controlled.
     *
     * @param spatial spatial to test (not null, unaffected)
     * @return true if the spatial is controlled by physics, otherwise false
     */
    public static boolean isPhysical(Spatial spatial) {
        Object rigidBodyControl = spatial.getControl(RigidBodyControl.class);
        boolean result = rigidBodyControl != null;

        return result;
    }

    /**
     * Read a spatial's mass.
     *
     * @param spatial which spatial to measure (not null, unaffected)
     * @return mass (&gt;0) or zero for a static object
     */
    public static float mass(Spatial spatial) {
        Validate.nonNull(spatial, "spatial");

        RigidBodyControl rigidBodyControl = findEnabledRbc(spatial);
        float mass = rigidBodyControl.getMass();

        assert mass >= 0f : mass;
        return mass;
    }

    /**
     * Remove all non-physics controls from the specified subtree of the scene
     * graph. Note: recursive!
     *
     * @param subtree (not null)
     */
    public static void removeNonPhysicsControls(Spatial subtree) {
        int numControls = subtree.getNumControls();
        for (int controlI = numControls - 1; controlI >= 0; --controlI) {
            Control control = subtree.getControl(controlI);
            if (!(control instanceof PhysicsControl)) {
                subtree.removeControl(control);
            }
        }
        if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                removeNonPhysicsControls(child);
            }
        }
    }

    /**
     * Alter whether the specified SGC applies physics coordinates to its
     * spatial's local translation.
     *
     * @param sgc control to alter (not null)
     * @param newSetting true means enable the control, false means disable it
     */
    public static void setApplyPhysicsLocal(Control sgc, boolean newSetting) {
        if (sgc instanceof AbstractPhysicsControl) {
            AbstractPhysicsControl apc = (AbstractPhysicsControl) sgc;
            apc.setApplyPhysicsLocal(newSetting);

        } else if (sgc instanceof GhostControl) {
            GhostControl gc = (GhostControl) sgc;
            gc.setApplyPhysicsLocal(newSetting);

        } else if (sgc instanceof RigidBodyControl) {
            RigidBodyControl rbc = (RigidBodyControl) sgc;
            rbc.setApplyPhysicsLocal(newSetting);

        } else if (sgc instanceof VehicleControl) {
            VehicleControl vc = (VehicleControl) sgc;
            vc.setApplyPhysicsLocal(newSetting);

        } else {
            String typeName = sgc.getClass().getCanonicalName();
            String msg = typeName + " does not support local physics.";
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter the enabled state of a scene-graph control.
     *
     * @param sgc control to alter (not null)
     * @param newState true means enable the control, false means disable it
     */
    public static void setEnabled(Control sgc, boolean newState) {
        if (sgc instanceof PhysicsControl) {
            PhysicsControl physicsControl = (PhysicsControl) sgc;
            physicsControl.setEnabled(newState);
        } else {
            MyControl.setEnabled(sgc, newState);
        }
    }
}
