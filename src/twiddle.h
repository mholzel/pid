#ifndef TWIDDLE_HEADER
#define TWIDDLE_HEADER

#include <iostream>
#include <vector>
#include <cmath>
#include <assert.h>
#include "Eigen/Dense"

using std::endl;

template<typename T>
T sum(const std::vector<T> &vector) {
    T s = 0;
    for (int i = 0; i < vector.size(); ++i)
        s += vector[i];
    return s;
}

template<typename T>
auto sum(const Eigen::MatrixBase<T> &matrix) -> decltype(matrix.sum()) {
    return matrix.sum();
}

/** This is the standard twiddle algorithm, which is useful when you have an
 * offline cost function that you just want to optimize. */
template<typename T>
std::vector<T>
twiddle(T (*cost_function)(const std::vector<T> &),
        const std::vector<T> &initial_guess,
        const std::vector<T> &perturbations,
        T tol = 1e-5,
        int max_iterations = 1e4) {

    /* The initial guess and perturbation vector must be the same size */
    assert(initial_guess.size() == perturbations.size());

    /* Make sure that the perturbations are positive */
    int n = perturbations.size();
    for (int i = 0; i < n; ++i) {
        perturbations[i] = std::abs(perturbations[i]);
    }

    /* Our initial best guess is just the initial guess */
    std::vector<T> parameters = initial_guess;
    T lowest_cost = cost_function(parameters);
    T cost = lowest_cost;

    /* Stop when the perturbations are small in every dimension
       Note that the list of perturbations will remain positive */
    int iteration = 0;
    while (iteration < max_iterations && sum(perturbations) > tol) {

        ++iteration;

        /* Perturb each axis of the parameter vector, one at a time */
        for (int i = 0; i < n; ++i) {

            parameters[i] += perturbations[i];
            cost = cost_function(parameters);

            /* If the perturbation was good, keep it, and increase the perturbation next time */
            if (cost < lowest_cost) {

                lowest_cost = cost;
                perturbations[i] *= 1.1;
            } else {

                /* Otherwise, try perturbing in the opposite direction */
                parameters[i] -= 2 * perturbations[i];
                cost = cost_function(parameters);

                /* If that worked, keep it, and increase the perturbation next time */
                if (cost < lowest_cost) {

                    lowest_cost = cost;
                    perturbations[i] *= 1.1;
                } else {

                    /* Otherwise, reset the value, and decrease the perturbation next time */
                    parameters[i] += perturbations[i];
                    perturbations[i] *= 0.9;
                }
            }
        }
    }
    return parameters;
}

/**
 * Sometimes Twiddle is easier to work with when the error calculations are triggered from another thread.
 * To avoid having to implement threading yourself, you can use this slightly modified form of twiddle.
 * The primary distinction is that this form of twiddle will only be updated when a new event occurs.
 * Specifically, when a new Event occurs, then you should pass it to the Twiddle class's update method.
 * The Parameters returned by the update method are the ones that Twiddle would like you to try in the next iteration.
 *
 * Internally, when you pass an Event to Twiddle's update method, twiddle will pass that event
 * to your cost function's update method. If this method returns a negative value, then we assume that
 * the cost function is not finished calculating. Otherwise, we assume that the returned value is the
 * cost associated with the current parameters.
 *
 *
 * a non-negative cost function object, which should indic
 */
template<typename CostFunction, typename CostValue, typename Parameters, typename Perturbations, typename Event>
class Twiddle {

private:
    CostFunction cost_function;
    Parameters parameters;
    Perturbations perturbations;
    const int max_iterations;
    CostValue tol;

    /* We need to keep track of the lowest cost observed to date */
    CostValue lowest_cost;

    /* The iteration count */
    int iteration = 1;

    /* This flag indicates that whether we have finished computing the cost
     * associated with the initial guess */
    bool initialized = false;

    /* This flag indicates whether the optimization has finished */
    bool finished = false;

    /* This index indicates the current index in the parameters that we are perturbing
     * that is, whose cost we are waiting to finish calculating */
    int index = -1;

    /* This bool indicates whether we are on the positive or negative perturbation */
    bool on_positive_branch = true;

    /** A method used to determine whether the optimization should stop */
    bool shouldStop() const {
        return index == parameters.size() && (iteration >= max_iterations || sum(perturbations) < tol);
    }

    /** We are done perturbing the parameter at the current index. Move on to the next. */
    void moveToNextParameter() {

        /* Actually move to the next index. */
        ++index;

        /* Check if we should stop */
        if (shouldStop()) {
            finished = true;
            std::cout << "Stopping. Iteration = " << iteration << ", max_iterations = " << max_iterations
                      << ", sum(perturbations) = " << sum(perturbations) << ", tol = " << tol << std::endl
                      << std::endl;
        } else {

            /* If we are not stopping, check if we were at the end of the parameter vector.
             * If we were, then we reset the index to 0 and increase the iteration count.*/
            if (index == parameters.size()) {
                index = 0;
                ++iteration;
            }
            parameters(index) += perturbations(index);
            on_positive_branch = true;
        }
    }

public:
    Twiddle(const CostFunction &cost_function,
            const Parameters &initial_guess,
            const Perturbations &perturbations,
            int max_iterations = 1e3,
            CostValue tol = 1e-5)
            : cost_function(cost_function),
              parameters(initial_guess),
              perturbations(perturbations),
              max_iterations(max_iterations),
              tol(tol) {
    }

    Parameters &update(const Event &event) {

        /* If the optimization has not finished, then continue optimizing */
        if (!finished) {

            /* First, let's see if the cost has finished computing for the current perturbation */
            CostValue cost = cost_function.update(event);
            if (cost >= 0) {

                std::cout << endl
                          << endl
                          << "Index " << index << ", Parameters: " << endl
                          << parameters.toString()
                          << endl
                          << endl;

                /* Check if we were in the initialization phase,
                 * where we are computing the cost for the initial guess. */
                if (!initialized) {

                    /* If we were initializing, then simply move to the next parameter */
                    initialized = true;
                    lowest_cost = cost;
                    moveToNextParameter();
                } else if (cost < lowest_cost) {

                    std::cout << "Cost decreased from " << lowest_cost << " to " << cost << endl
                              << endl;

                    /* If the cost decreased, then we can move on and perturb the next parameter,
                     * increasing the perturbation amount for the current parameter in the next iteration. */
                    lowest_cost = cost;
                    perturbations(index) *= 2;
                    moveToNextParameter();
                } else if (on_positive_branch) {

                    std::cout << "Positive branch failed. Moving to negative branch " << endl
                              << endl;

                    /* If the positive branch did not yield a good perturbation,
                     * then we try subtracting the perturbation instead. */
                    on_positive_branch = false;
                    parameters(index) -= 2 * perturbations(index);
                } else {

                    std::cout << "Negative branch failed. Resetting, then moving to next index. " << endl
                              << endl;

                    /* Neither the positive nor negative branch decreased the cost.
                     * So reset the value, decrease the perturbation for the next iteration,
                     * and move on to the next parameter */
                    parameters(index) += perturbations(index);
                    perturbations(index) *= 1 / sqrt(5.0);
                    moveToNextParameter();
                }

                /* Reset any internal values in the parameters */
                parameters.reset();
            }
        }
        return parameters;
    }
};

#endif /* TWIDDLE_HEADER */
