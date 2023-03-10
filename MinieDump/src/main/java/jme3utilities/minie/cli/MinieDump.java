/*
 Copyright (c) 2021-2023, Stephen Gold
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
package jme3utilities.minie.cli;

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.scene.Spatial;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.texture.plugins.AWTLoader;
import java.util.logging.Logger;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;

/**
 * A command-line utility to dump J3O assets.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MinieDump {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(MinieDump.class.getName());
    // *************************************************************************
    // fields

    /**
     * load assets
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    /**
     * dump asset descriptions to {@code System.out}
     */
    final private static PhysicsDumper dumper = new PhysicsDumper();
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MinieDump() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MinieDump application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        setupAssetManager();
        setupNativeLibrary();

        // Process the command-line arguments.
        for (String argument : arguments) {
            if (argument.equals("--verbose") || argument.equals("-v")) {
                dumper.setEnabled(DumpFlags.ChildShapes, true);
                dumper.setEnabled(DumpFlags.MatParams, true);
            } else if (argument.endsWith(".j3o")) {
                dumpAsset(argument);
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Dump the asset at the specified path.
     *
     * @param assetPath a path to the asset (not null, not empty)
     */
    private static void dumpAsset(String assetPath) {
        System.out.print(assetPath);
        System.out.print(" contains a ");
        System.out.flush();

        Object loadedAsset;
        try {
            loadedAsset = assetManager.loadAsset(assetPath);

        } catch (IllegalStateException exception) {
            /*
             * Perhaps the asset is a C-G model,
             * in which case it needs an AssetProcessor to handle cloning.
             */
            ModelKey modelKey = new ModelKey(assetPath);
            loadedAsset = assetManager.loadAsset(modelKey);
        }

        if (loadedAsset instanceof CollisionShape) {
            System.out.print("collision shape:");
            dumper.dump((CollisionShape) loadedAsset, "  ");
            System.out.println();
            System.out.println();

        } else if (loadedAsset instanceof Spatial) {
            System.out.println("model:");
            dumper.dump((Spatial) loadedAsset, "  ");
            System.out.println();
            System.out.println();

        } else {
            String className = loadedAsset.getClass().getSimpleName();
            System.out.print(className);
            System.out.println('.');
        }
    }

    /**
     * Configure the AssetManager.
     */
    private static void setupAssetManager() {
        // Register loaders.
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");

        // Register locators.
        assetManager.registerLocator(".", FileLocator.class);
        assetManager.registerLocator(null, ClasspathLocator.class);
    }

    /**
     * Load and configure the native library for this platform.
     */
    private static void setupNativeLibrary() {
        // Don't extract to the working directory!
        NativeLibraryLoader.setCustomExtractionFolder("/tmp");

        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        NativeLibrary.setStartupMessageEnabled(false);
    }
}
