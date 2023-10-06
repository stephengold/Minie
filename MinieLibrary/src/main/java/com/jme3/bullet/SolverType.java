/*
 * Copyright (c) 2020 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet;

/**
 * Enumerate the available contact-and-constraint solvers.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public enum SolverType {
    // *************************************************************************
    // values

    /**
     * btSequentialImpulseConstraintSolver/btMultiBodyConstraintSolver: Bullet's
     * original sequential-impulse solver
     */
    SI,
    /**
     * btDantzigSolver: Mixed Linear Complementarity Problem (MLCP) direct
     * solver using the Dantzig Algorithm
     */
    Dantzig,
    /**
     * btLemkeSolver: accurate-but-slow MLCP direct solver using Lemke’s
     * Algorithm (see "Fast Implementation of Lemke’s Algorithm for Rigid Body
     * Contact Simulation" by John E. Lloyd)
     * <p>
     * Seems to require a global CFM &gt; 0.
     */
    Lemke,
    /**
     * btSolveProjectedGaussSeidel: slow MLCP direct solver using projected
     * Gauss-Seidel (PGS) for debug/learning purposes
     */
    PGS,
    /**
     * btNNCGConstraintSolver: using the Non-smooth Nonlinear Conjugate Gradient
     * (NNCG) method
     */
    NNCG
}
