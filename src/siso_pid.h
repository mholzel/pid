#ifndef SISO_PID_HEADER
#define SISO_PID_HEADER

#include <sstream>
#include <string>

template<typename T = double>
class SisoPID {
private:

    /* Errors */
    T last_error;
    T integral_of_errors;

    /* Coefficients */
    const T k_p;
    const T k_i;
    const T k_d;

public:

    /** Constructors */
    SisoPID(T k_p, T k_i, T k_d) : k_p(k_p), k_i(k_i), k_d(k_d) {}

    /** Destructor */
    virtual ~SisoPID() {}

    /** Compute the PID output and update the PID state */
    T update(T error, double dt = 1) {
        T derivative_of_error = (error - last_error) / dt;
        integral_of_errors += dt * (last_error + (error - last_error) / 2);
        last_error = error;
        return -k_p * error - k_d * derivative_of_error - k_i * integral_of_errors;
    }

    /** Convert the parameters to a comman-separated list string in the form k_p,k_i,k_d */
    std::string toString() {
        std::stringstream cout;
        cout << k_p << "," << k_i << "," << k_d << std::endl;
        return cout.str();
    }

    /** Overload the () operator so that the coefficients can be accessed via linear indexing */
    T &operator()(int index) {
        if (index == 0) {
            return k_p;
        } else if (index == 1) {
            return k_i;
        } else if (index == 2) {
            return k_d;
        } else {
            assert(false);
        }
    }
};

#endif /* SISO_PID_HEADER */
