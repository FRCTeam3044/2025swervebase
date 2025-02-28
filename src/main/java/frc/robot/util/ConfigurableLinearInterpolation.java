package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;

import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxconfig.OxConfig;

public class ConfigurableLinearInterpolation implements ConfigurableClass {
    ConfigurableClassParam<Double> x1;
    ConfigurableClassParam<Double> y1;
    ConfigurableClassParam<Double> x2;
    ConfigurableClassParam<Double> y2;
    private String key;
    private String prettyName;
    private final ArrayList<ConfigurableClassParam<?>> params = new ArrayList<>();

    public ConfigurableLinearInterpolation(String key) {
        this.key = key;
        this.prettyName = key;
        x1 = new ConfigurableClassParam<Double>(this, 0.0, (x) -> {
        }, "x1");
        y1 = new ConfigurableClassParam<Double>(this, 0.0, (x) -> {
        }, "y1");
        x2 = new ConfigurableClassParam<Double>(this, 0.0, (x) -> {
        }, "x2");
        y2 = new ConfigurableClassParam<Double>(this, 0.0, (x) -> {
        }, "y2");
        Collections.addAll(params, x1, y1, x2, y2);
        OxConfig.registerConfigurableClass(this);

    }

    @Override
    public ArrayList<ConfigurableClassParam<?>> getParameters() {
        return params;
    }

    @Override
    public String getKey() {
        return key;
    }

    @Override
    public String getPrettyName() {
        return prettyName;
    }

    public double calculate(double x) {
        return y1.get() + (y2.get() - y1.get()) * (x - x1.get()) / (x2.get() - x1.get());
    }

    public double getY1() {
        return y1.get();
    }

    public double getY2() {
        return y2.get();
    }

    public double getX1() {
        return x1.get();
    }

    public double getX2() {
        return x2.get();
    }
}
