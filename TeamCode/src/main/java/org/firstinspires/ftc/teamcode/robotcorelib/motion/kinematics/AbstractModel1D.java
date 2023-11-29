package org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BisectionSolver;

public class AbstractModel1D {

    private final UnivariateFunction function;
    private final double min, max;
    private final double minInv, maxInv;
    private final BisectionSolver solver = new BisectionSolver();

    public AbstractModel1D(UnivariateFunction function, double min, double max) {
        this.function = function;
        this.min = min;
        this.max = max;
        minInv = function.value(min);
        maxInv = function.value(max);
    }

    public double inverse(double y) {
        y = Range.clip(y, minInv, maxInv);
        return solver.solve(1000, new InverseUnivariateFunctionSolver(function, y), min, max);
    }

    public double value(double x) {
        return function.value(x);
    }

    static class InverseUnivariateFunctionSolver implements UnivariateFunction {
        private final UnivariateFunction function;
        private final double inverseValue;

        public InverseUnivariateFunctionSolver(UnivariateFunction function, double inverseValue) {
            this.function = function;
            this.inverseValue = inverseValue;
        }

        @Override
        public double value(double x) {
            return function.value(x) - inverseValue;
        }
    }

}
