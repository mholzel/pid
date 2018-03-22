#ifndef MIMO_PID_HEADER
#define MIMO_PID_HEADER

#include <sstream>
#include <string>
#include "Eigen/Dense"

template <typename T = double, int n_controls = 1, int n_measurements = 1>
class MimoPID
{
  private:
    /* Errors */
    using Error = Eigen::Matrix<T, n_measurements, 1>;
    Error last_error;
    Error integral_of_errors;

    /* Coefficients */
    using Gain = Eigen::Matrix<T, n_controls, n_measurements>;
    Gain k_p;
    Gain k_i;
    Gain k_d;

    using Control = Eigen::Matrix<T, n_controls, 1>;

    static const int n_parameters_per_gain = n_controls * n_measurements;
    static const int n_parameters = 3 * n_parameters_per_gain;

  public:

    bool was_reset = false;

    /** Constructors */
    MimoPID(const Gain &k_p,
            const Gain &k_i,
            const Gain &k_d)
        : k_p(k_p),
          k_i(k_i),
          k_d(k_d)
    {
        last_error = Error::Zero();
        integral_of_errors = Error::Zero();
    }

    /** Destructor */
    virtual ~MimoPID() {}

    /** Compute the PID output and update the PID state */
    Control update(Error &error, double dt = 1)
    {
        Error derivative_of_error = (error - last_error) / dt;
        integral_of_errors += dt * (last_error + (error - last_error) / 2);
        last_error = error;
        return -k_p * error - k_d * derivative_of_error - k_i * integral_of_errors;
    }

    /** Convert the parameters to a comman-separated list string in the form k_p,k_i,k_d */
    std::string toString()
    {
        std::stringstream cout;
        cout << k_p << "," << k_i << "," << k_d << std::endl;
        return cout.str();
    }

    /** Overload the () operator so that the coefficients can be accessed via linear indexing.
     * This is used primarily by the optimization algorithms */
    T &operator()(int index)
    {
        if (index < n_parameters_per_gain)
        {
            return k_p(index);
        }
        else if (index < 2 * n_parameters_per_gain)
        {
            return k_i(index - n_parameters_per_gain);
        }
        else if (index < 3 * n_parameters_per_gain)
        {
            return k_d(index - 2 * n_parameters_per_gain);
        }
        else
        {
            assert(false);
        }
    }

    int size() const
    {
        return n_parameters;
    }

    void reset()
    {
        last_error = Error::Zero();
        integral_of_errors = Error::Zero();
        was_reset = true;
    }
};

#endif /* MIMO_PID_HEADER */
