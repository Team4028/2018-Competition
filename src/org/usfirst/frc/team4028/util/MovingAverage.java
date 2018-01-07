package org.usfirst.frc.team4028.util;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.LinkedList;
import java.util.Queue;

// this class maintains a moving average for its contents
public class MovingAverage {
    private final Queue<BigDecimal> window = new LinkedList<BigDecimal>();
    private final int period;
    private BigDecimal sum = BigDecimal.ZERO;
    
    public MovingAverage(int period) {
        assert period > 0 : "Period must be a positive integer";
        this.period = period;
    }

    // add a sample (optional remove oldest entry if internal queue is already at target size)
    public void add(BigDecimal num) {
        sum = sum.add(num);
        window.add(num);
        if (window.size() > period) {
            sum = sum.subtract(window.remove());
        }
    }

    public BigDecimal getAverage() {
        if (window.isEmpty()) return BigDecimal.ZERO; // technically the average is undefined
        BigDecimal divisor = BigDecimal.valueOf(window.size());
        return sum.divide(divisor, 2, RoundingMode.HALF_UP);
    }
}