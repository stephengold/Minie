/*
 Copyright (c) 2014-2018, Stephen Gold
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
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.math.Vector3f;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * Utility methods for physics collision shapes. All methods should be static.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyShape.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyShape() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the main axis of the specified shape, provided it's a capsule,
     * cone, or cylinder.
     *
     * @param shape (may be null, unaffected)
     * @return 0&rarr;X, 1&rarr;Y, 2&rarr;Z, -1&rarr;doesn't have an axis
     */
    public static int axisIndex(CollisionShape shape) {
        int result = -1;
        if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            result = capsule.getAxis();

        } else if (shape instanceof ConeCollisionShape) {
            ConeCollisionShape cone = (ConeCollisionShape) shape;
            result = cone.getAxis();

        } else if (shape instanceof CylinderCollisionShape) {
            CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
            result = cylinder.getAxis();
        }

        return result;
    }

    /**
     * Generate a brief textual description of a shape, consisting of its type
     * and id.
     *
     * @param shape instance to describe (not null, unaffected)
     * @return description (not null, not empty)
     */
    public static String describe(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        String type = describeType(shape);
        type = type.toLowerCase(Locale.ROOT);
        long id = shape.getObjectId();
        String result = String.format("%s:%x", type, id);

        return result;
    }

    /**
     * Describe the type of a shape.
     *
     * @param shape instance to describe (not null, unaffected)
     * @return description (not null)
     */
    public static String describeType(CollisionShape shape) {
        String description = shape.getClass().getSimpleName();
        if (description.endsWith("CollisionShape")) {
            description = MyString.removeSuffix(description, "CollisionShape");
        }

        return description;
    }

    /**
     * Calculate the un-scaled half extents of the specified shape, which must
     * be a box, capsule, cone, cylinder, or sphere.
     *
     * @param shape (not null, unaffected)
     * @param storeResult (modified if not null)
     * @return a vector with all components non-negative (either storeResult or
     * a new instance)
     */
    public static Vector3f halfExtents(CollisionShape shape,
            Vector3f storeResult) {
        Validate.nonNull(shape, "shape");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        if (shape instanceof BoxCollisionShape) {
            BoxCollisionShape box = (BoxCollisionShape) shape;
            box.getHalfExtents(result);

        } else if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            float height = capsule.getHeight();
            float radius = capsule.getRadius();
            float axisHalfExtent = height / 2f + radius;
            int axisIndex = axisIndex(shape);
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    result.set(axisHalfExtent, radius, radius);
                    break;
                case PhysicsSpace.AXIS_Y:
                    result.set(radius, axisHalfExtent, radius);
                    break;
                case PhysicsSpace.AXIS_Z:
                    result.set(radius, radius, axisHalfExtent);
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }

        } else if (shape instanceof ConeCollisionShape) {
            ConeCollisionShape cone = (ConeCollisionShape) shape;
            float height = cone.getHeight();
            float radius = cone.getRadius();
            float axisHalfExtent = height / 2f;
            int axisIndex = axisIndex(shape);
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    result.set(axisHalfExtent, radius, radius);
                    break;
                case PhysicsSpace.AXIS_Y:
                    result.set(radius, axisHalfExtent, radius);
                    break;
                case PhysicsSpace.AXIS_Z:
                    result.set(radius, radius, axisHalfExtent);
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }

        } else if (shape instanceof CylinderCollisionShape) {
            CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
            cylinder.getHalfExtents(result);

        } else if (shape instanceof HullCollisionShape) {
            HullCollisionShape hull = (HullCollisionShape) shape;
            hull.getHalfExtents(result);

        } else if (shape instanceof MultiSphere) {
            MultiSphere multiSphere = (MultiSphere) shape;
            if (multiSphere.countSpheres() == 1) {
                float radius = multiSphere.getRadius(0);
                result.set(radius, radius, radius);
            }

        } else if (shape instanceof SimplexCollisionShape) {
            SimplexCollisionShape simplex = (SimplexCollisionShape) shape;
            simplex.getHalfExtents(result);

        } else if (shape instanceof SphereCollisionShape) {
            SphereCollisionShape sphere = (SphereCollisionShape) shape;
            float radius = sphere.getRadius();
            result.set(radius, radius, radius);

        } else { // TODO handle more shapes
            String typeName = shape.getClass().getCanonicalName();
            String msg = String.format("%s lacks half extents.", typeName);
            throw new IllegalArgumentException(msg);
        }

        assert MyVector3f.isAllNonNegative(result) : result;

        return result;
    }

    /**
     * Calculate the un-scaled height of the specified shape.
     *
     * @param shape (not null, unaffected)
     * @return un-scaled height &ge;0) or NaN if the shape is not a capsule,
     * cone, cylinder, or sphere
     */
    public static float height(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        float result = Float.NaN;
        if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            result = capsule.getHeight();

        } else if (shape instanceof ConeCollisionShape) {
            ConeCollisionShape cone = (ConeCollisionShape) shape;
            result = cone.getHeight();

        } else if (shape instanceof CylinderCollisionShape) {
            Vector3f halfExtents = halfExtents(shape, null);
            int axisIndex = axisIndex(shape);
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    result = halfExtents.x;
                    break;
                case PhysicsSpace.AXIS_Y:
                    result = halfExtents.y;
                    break;
                case PhysicsSpace.AXIS_Z:
                    result = halfExtents.z;
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }

        } else if (shape instanceof MultiSphere) {
            MultiSphere multiSphere = (MultiSphere) shape;
            if (multiSphere.countSpheres() == 1) {
                result = 2f * multiSphere.getRadius(0);
            }

        } else if (shape instanceof SphereCollisionShape) {
            SphereCollisionShape sphere = (SphereCollisionShape) shape;
            result = 2f * sphere.getRadius();
        }

        assert Float.isNaN(result) || result >= 0f : result;
        return result;
    }

    /**
     * Parse the id of a shape from its description.
     *
     * @param description input text (not null, not empty)
     * @return the shape's id
     *
     * @see #describe(com.jme3.bullet.collision.shapes.CollisionShape)
     */
    public static long parseShapeId(String description) {
        Validate.nonEmpty(description, "description");

        String[] parts = description.split(":");
        assert parts.length == 2 : parts.length;
        String hexadecimal = parts[1];
        long result = Long.parseLong(hexadecimal, 16);

        return result;
    }

    /**
     * Calculate the un-scaled radius of the specified shape.
     *
     * @param shape (not null, unaffected)
     * @return un-scaled radius (&ge;0) or NaN if the radius is undefined or
     * unknown
     */
    public static float radius(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        float result = Float.NaN;
        if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            result = capsule.getRadius();

        } else if (shape instanceof ConeCollisionShape) {
            ConeCollisionShape cone = (ConeCollisionShape) shape;
            result = cone.getRadius();

        } else if (shape instanceof CylinderCollisionShape) {
            Vector3f halfExtents = halfExtents(shape, null);
            int axisIndex = axisIndex(shape);
            float r1, r2;
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    r1 = halfExtents.y;
                    r2 = halfExtents.z;
                    break;
                case PhysicsSpace.AXIS_Y:
                    r1 = halfExtents.x;
                    r2 = halfExtents.z;
                    break;
                case PhysicsSpace.AXIS_Z:
                    r1 = halfExtents.x;
                    r2 = halfExtents.y;
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }
            if (r1 == r2) {
                result = r1;
            }

        } else if (shape instanceof MultiSphere) {
            MultiSphere multiSphere = (MultiSphere) shape;
            if (multiSphere.countSpheres() == 1) {
                result = multiSphere.getRadius(0);
            }

        } else if (shape instanceof SphereCollisionShape) {
            SphereCollisionShape sphere = (SphereCollisionShape) shape;
            result = sphere.getRadius();
        }

        assert Float.isNaN(result) || result >= 0f : result;
        return result;
    }

    /**
     * Copy a shape, altering only its half extents.
     *
     * @param oldShape input shape (not null, unaffected)
     * @param newHalfExtents (not null, no negative component, unaffected)
     * @return a new shape, or null if not possible
     */
    public static CollisionShape setHalfExtents(CollisionShape oldShape,
            Vector3f newHalfExtents) {
        Validate.nonNull(oldShape, "old shape");
        Validate.nonNegative(newHalfExtents, "new half extents");

        CollisionShape result;
        if (oldShape instanceof BoxCollisionShape) {
            result = new BoxCollisionShape(newHalfExtents);

        } else if (oldShape instanceof CapsuleCollisionShape
                || oldShape instanceof ConeCollisionShape) {
            int axisIndex = axisIndex(oldShape);
            float axisHalfExtent, radius1, radius2;
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    axisHalfExtent = newHalfExtents.x;
                    radius1 = newHalfExtents.y;
                    radius2 = newHalfExtents.z;
                    break;
                case PhysicsSpace.AXIS_Y:
                    axisHalfExtent = newHalfExtents.y;
                    radius1 = newHalfExtents.x;
                    radius2 = newHalfExtents.z;
                    break;
                case PhysicsSpace.AXIS_Z:
                    axisHalfExtent = newHalfExtents.z;
                    radius1 = newHalfExtents.x;
                    radius2 = newHalfExtents.y;
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }
            if (radius1 != radius2) {
                result = null;
            } else if (oldShape instanceof CapsuleCollisionShape) {
                float height = 2f * (axisHalfExtent - radius1);
                result = new CapsuleCollisionShape(radius1, height, axisIndex);
            } else {
                assert oldShape instanceof ConeCollisionShape;
                float height = 2f * axisHalfExtent;
                result = new ConeCollisionShape(radius1, height, axisIndex);
            }

        } else if (oldShape instanceof CylinderCollisionShape) {
            int axisIndex = axisIndex(oldShape);
            result = new CylinderCollisionShape(newHalfExtents, axisIndex);

        } else if (oldShape instanceof SphereCollisionShape) {
            if (!MyVector3f.isScaleUniform(newHalfExtents)) {
                result = null;
            } else {
                result = new SphereCollisionShape(newHalfExtents.x);
            }

        } else {
            result = null;
        }

        if (result != null
                && !(result instanceof CapsuleCollisionShape)
                && !(result instanceof SphereCollisionShape)) {
            float margin = oldShape.getMargin();
            result.setMargin(margin);
        }

        return result;
    }

    /**
     * Copy a shape, altering only its height.
     *
     * @param oldShape input shape (not null, unaffected)
     * @param newHeight un-scaled height (&ge;0)
     * @return a new shape
     */
    public static CollisionShape setHeight(CollisionShape oldShape,
            float newHeight) {
        Validate.nonNull(oldShape, "old shape");
        Validate.nonNegative(newHeight, "new height");

        CollisionShape result;
        if (oldShape instanceof BoxCollisionShape) {
            result = setRadius(oldShape, newHeight / 2f);

        } else if (oldShape instanceof CapsuleCollisionShape) {
            float radius = radius(oldShape);
            int axisIndex = axisIndex(oldShape);
            result = new CapsuleCollisionShape(radius, newHeight, axisIndex);

        } else if (oldShape instanceof ConeCollisionShape) {
            float radius = radius(oldShape);
            int axisIndex = axisIndex(oldShape);
            result = new ConeCollisionShape(radius, newHeight, axisIndex);

        } else if (oldShape instanceof CylinderCollisionShape) {
            Vector3f halfExtents = halfExtents(oldShape, null);
            int axisIndex = axisIndex(oldShape);
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    halfExtents.x = newHeight;
                    break;
                case PhysicsSpace.AXIS_Y:
                    halfExtents.y = newHeight;
                    break;
                case PhysicsSpace.AXIS_Z:
                    halfExtents.z = newHeight;
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }
            result = new CylinderCollisionShape(halfExtents, axisIndex);

        } else if (oldShape instanceof SphereCollisionShape) {
            result = setHeight(oldShape, newHeight / 2f);

        } else {
            result = null;
        }

        if (result != null
                && !(result instanceof CapsuleCollisionShape)
                && !(result instanceof SphereCollisionShape)) {
            float margin = oldShape.getMargin();
            result.setMargin(margin);
        }

        return result;
    }

    /**
     * Copy a shape, altering only its radius.
     *
     * @param oldShape input shape (not null, unaffected)
     * @param newRadius un-scaled radius (&ge;0)
     * @return a new shape
     */
    public static CollisionShape setRadius(CollisionShape oldShape,
            float newRadius) {
        Validate.nonNull(oldShape, "old shape");
        Validate.nonNegative(newRadius, "new radius");

        CollisionShape result;
        if (oldShape instanceof BoxCollisionShape) {
            Vector3f halfExtents
                    = new Vector3f(newRadius, newRadius, newRadius);
            result = new BoxCollisionShape(halfExtents);

        } else if (oldShape instanceof CapsuleCollisionShape) {
            int axisIndex = axisIndex(oldShape);
            float height = height(oldShape);
            result = new CapsuleCollisionShape(newRadius, height, axisIndex);

        } else if (oldShape instanceof ConeCollisionShape) {
            int axisIndex = axisIndex(oldShape);
            float height = height(oldShape);
            result = new ConeCollisionShape(newRadius, height, axisIndex);

        } else if (oldShape instanceof CylinderCollisionShape) {
            Vector3f halfExtents = halfExtents(oldShape, null);
            int axisIndex = axisIndex(oldShape);
            switch (axisIndex) {
                case PhysicsSpace.AXIS_X:
                    halfExtents.y = newRadius;
                    halfExtents.z = newRadius;
                    break;
                case PhysicsSpace.AXIS_Y:
                    halfExtents.x = newRadius;
                    halfExtents.z = newRadius;
                    break;
                case PhysicsSpace.AXIS_Z:
                    halfExtents.x = newRadius;
                    halfExtents.y = newRadius;
                    break;
                default:
                    String msg = Integer.toString(axisIndex);
                    throw new IllegalStateException(msg);
            }
            result = new CylinderCollisionShape(halfExtents, axisIndex);

        } else if (oldShape instanceof SphereCollisionShape) {
            result = new SphereCollisionShape(newRadius);

        } else {
            result = null;
        }

        if (result != null
                && !(result instanceof CapsuleCollisionShape)
                && !(result instanceof SphereCollisionShape)) {
            float margin = oldShape.getMargin();
            result.setMargin(margin);
        }

        return result;
    }

    /**
     * Compute the volume of a closed collision shape.
     *
     * @param shape (not null, unaffected)
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    public static float volume(CollisionShape shape) {
        Vector3f scale = shape.getScale(null);
        float volume = scale.x * scale.y * scale.z;

        if (shape instanceof BoxCollisionShape) {
            BoxCollisionShape box = (BoxCollisionShape) shape;
            Vector3f halfExtents = box.getHalfExtents(null);
            volume *= MyVolume.boxVolume(halfExtents);

        } else if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            float height = capsule.getHeight();
            float radius = capsule.getRadius();
            volume *= MyVolume.capsuleVolume(radius, height);

        } else if (shape instanceof CompoundCollisionShape) {
            /*
             * CompoundCollisionShape ignores scaling.  This calculation
             * also ignores any overlaps between the children. TODO test this
             */
            CompoundCollisionShape compound = (CompoundCollisionShape) shape;
            volume = 0f;
            for (ChildCollisionShape child : compound.getChildren()) {
                float childVolume = volume(child.getShape());
                volume += childVolume;
            }

        } else if (shape instanceof ConeCollisionShape) {
            ConeCollisionShape cone = (ConeCollisionShape) shape;
            float radius = cone.getRadius();
            float height = cone.getHeight();
            volume *= MyVolume.coneVolume(radius, height);

        } else if (shape instanceof CylinderCollisionShape) {
            CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
            Vector3f halfExtents = cylinder.getHalfExtents(null);
            volume *= MyVolume.cylinderVolume(halfExtents);

        } else if (shape instanceof HullCollisionShape) {
            HullCollisionShape hull = (HullCollisionShape) shape;
            volume = hull.scaledVolume();

        } else if (shape instanceof MultiSphere) {
            MultiSphere multiSphere = (MultiSphere) shape;
            volume = multiSphere.scaledVolume();

        } else if (shape instanceof SimplexCollisionShape) {
            SimplexCollisionShape simplex = (SimplexCollisionShape) shape;
            volume *= simplex.unscaledVolume();

        } else if (shape instanceof SphereCollisionShape) {
            SphereCollisionShape sphere = (SphereCollisionShape) shape;
            float radius = sphere.getRadius();
            volume *= MyVolume.sphereVolume(radius);

        } else {
            logger.log(Level.SEVERE, "shape={0}", shape.getClass());
            throw new IllegalArgumentException("Shape must be closed!");
        }

        assert volume >= 0f : volume;
        return volume;
    }
}
