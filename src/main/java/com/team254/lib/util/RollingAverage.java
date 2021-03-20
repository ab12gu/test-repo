package com.team254.lib.util;

import java.util.ArrayList;

public class RollingAverage {
    private ArrayList<Double> list;
    private int maxSamples = 0;

    public RollingAverage(int maxSamples) {
        list = new ArrayList<>();
        this.maxSamples = maxSamples;
    }

    public void addObservation(double observation) {
        if (list.size() < maxSamples) {
            list.add(observation);
        } else {
            list.remove(0);
            list.add(observation);
        }
    }

    public double getValue() {
        double count = 0;
        for (double i : list) {
            count += i;
        }
        return count / list.size();
    }
 }
