/*
 Copyright (c) 2019-2023, Stephen Gold
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
package jme3utilities.minie.wizard;

import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.renderer.RenderManager;
import java.util.logging.Logger;
import jme3utilities.nifty.bind.BindScreen;
import jme3utilities.nifty.displaysettings.DsScreen;
import jme3utilities.ui.InputMode;

/**
 * Action strings for the DacWizard application. Each String describes a
 * user-interface action. By convention, action strings begin with a verb in all
 * lowercase and never end with a space (' ').
 *
 * @author Stephen Gold sgold@sonic.net
 */
final class Action {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(Action.class.getName());

    final static String browse = "browse";
    final static String dumpAppStates = "dump appStates";
    final static String dumpPhysicsSpace = "dump physicsSpace";
    final static String dumpRenderer = "dump renderer";
    final static String editBindings = "edit bindings";
    final static String editDisplaySettings = "edit displaySettings";
    final static String editLinks = "edit links";
    final static String load = "load";
    final static String morePath = "more path";
    final static String moreRoot = "more root";
    final static String nextAnimation = "next animation";
    final static String nextMassHeuristic = "next massHeuristic";
    final static String nextScreen = "next screen";
    final static String pickLink = "pick link";
    final static String previousAnimation = "previous animation";
    final static String previousScreen = "previous screen";
    final static String save = "save";
    final static String saveJ3o = "saveJ3o";
    final static String selectCenterHeuristic = "select centerHeuristic";
    final static String selectRotationOrder = "select rotationOrder";
    final static String selectShapeHeuristic = "select shapeHeuristic";
    final static String setAnimationTime = "set animationTime";
    final static String setMargin = "set margin";
    final static String setMassParameter = "set massParameter";
    final static String setShapeScale = "set shapeScale";
    final static String toggleAngleMode = "toggle angleMode";
    final static String toggleAxes = "toggle axes";
    final static String toggleMesh = "toggle mesh";
    final static String togglePhysicsDebug = "toggle physicsDebug";
    final static String toggleRagdoll = "toggle ragdoll";
    final static String toggleSkeleton = "toggle skeleton";
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Action() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Process an ongoing action from the GUI or keyboard that wasn't handled by
     * the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @return true if the action has been handled, otherwise false
     */
    static boolean processOngoing(String actionString) {
        boolean handled = true;

        Model model = DacWizard.getModel();
        switch (actionString) {
            case dumpAppStates:
                dumpAppStates();
                break;

            case dumpPhysicsSpace:
                dumpPhysicsSpace();
                break;

            case dumpRenderer:
                dumpRenderer();
                break;

            case editBindings:
                editBindings();
                break;

            case editDisplaySettings:
                editDisplaySettings();
                break;

            case nextAnimation:
                model.nextAnimation();
                break;

            case previousAnimation:
                model.previousAnimation();
                break;

            default:
                handled = false;
        }

        return handled;
    }
    // *************************************************************************
    // private methods

    /**
     * Process a "dump appStates" action.
     */
    private static void dumpAppStates() {
        DacWizard app = DacWizard.getApplication();
        AppStateManager stateManager = app.getStateManager();
        DacWizard.dumper.dump(stateManager);
    }

    /**
     * Process a "dump physicsSpace" action.
     */
    private static void dumpPhysicsSpace() {
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        DacWizard.dumper.dump(physicsSpace);
    }

    /**
     * Process a "dump renderer" action.
     */
    private static void dumpRenderer() {
        DacWizard app = DacWizard.getApplication();
        RenderManager renderManager = app.getRenderManager();
        DacWizard.dumper.dump(renderManager);
    }

    /**
     * Process an "edit bindings" action.
     */
    private static void editBindings() {
        BindScreen bindScreen = DacWizard.findAppState(BindScreen.class);
        InputMode active = InputMode.getActiveMode();
        bindScreen.activate(active);
    }

    /**
     * Process an "edit displaySettings" action.
     */
    private static void editDisplaySettings() {
        DsScreen dss = DacWizard.findAppState(DsScreen.class);
        dss.activate();
    }
}
