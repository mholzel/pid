#ifndef TELEMETRY_HEADER
#define TELEMETRY_HEADER

template<typename T>
class Telemetry {
public:
    const T crosstrack_error;
    const T speed;
    const T angle;
    const T throttle;
    const T dt;

    Telemetry(const T &crosstrack_error,
              const T &speed,
              const T &angle,
              const T &throttle,
              const T &dt)
            : crosstrack_error(crosstrack_error),
              speed(speed),
              angle(angle),
              throttle(throttle),
              dt(dt) {
    }

    virtual ~Telemetry() {}
};

#endif /* TELEMETRY_HEADER */
