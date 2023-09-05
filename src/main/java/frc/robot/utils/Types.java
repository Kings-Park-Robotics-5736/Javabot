package frc.robot.utils;


/**
 * @brief A class for creating convenience type wrappers to allow us to more easily pass around values.
 */
public final class Types {
  public static final class PidConstants {
    public  double p; 
    public  double i;
    public  double d;

    public PidConstants(double _p, double _i, double _d) {
      p = _p;
      i = _i;
      d = _d;
    }
  }

  public static final class FeedForwardConstants {
    public final double ks;
    public final double kv;
    public final double ka;
    public final double kg;

    public FeedForwardConstants(double _ks, double _kv, double _ka) {
      this(_ks, _kv, _ka, 0);
    }
    public FeedForwardConstants(double _ks, double _kv, double _ka, double _kg) {
      ks = _ks;
      kv = _kv;
      ka = _ka;
      kg = _kg;
    }
  }

  public static final class Limits {
    public final double low;
    public final double high;


    public Limits(double low, double high) {
      this.low = low;
      this.high = high;
    }
  }

  public enum DirectionType{
    UP,
    DOWN
  };
}
