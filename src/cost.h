#ifndef COST_HEADER
#define COST_HEADER

#include "telemetry.h"
#include "math.h"

template<typename T>
class Cost {

private:

    const int iterations;
    const int skip_iterations;
    T cost;
    int iteration;
    T lowest_cost;

    /**
     * If we are under the iteration count AND either
     * we haven't computed a lowest cost yet OR
     * the average cost might be lower than the previously computed lowest cost,
     * then continue.
     */
    bool shouldContinue() {
        return iteration < iterations && (lowest_cost < 0 || (cost / iterations < lowest_cost));
    }

public:

    Cost(int iterations = 200,
         int skip_iterations = 50)
            : iterations(iterations),
              skip_iterations(skip_iterations) {
        reset();
        lowest_cost = -1;
    }

    virtual ~Cost() {}

    T update(const Telemetry<T> &telemetry) {

        /* Update the cost and iteration count.
         * Note that we don't include the cost during the first few iterations
         * because the previous controller might have put the car in a bad position.
         * So a good controller should be given time to correct that for that bad initial error. */
        ++iteration;
        if (iteration > skip_iterations)
            cost += pow(telemetry.crosstrack_error, 2) / telemetry.speed;

        /* We need to check if we should stop computing the cost for a given set of parameters.
         * This should happen if we have hit the number of iterations that we want to test.
         * OR if we can already tell that this set of parameters will yield a higher average cost
         * than the lowest cost we have found so far.
         *
         * If we should continue, then we signal that by returning a negative number.
         * Otherwise, if we should stop, then we return the average cost for the the specified parameters. */
        if (shouldContinue()) {

            return -1;

        } else {

            /* We are done computing the cost for a particular set of parameters.
             * If this is the new lowest average cost, then save it. */
            T average_cost = cost / iteration;
            if (lowest_cost < 0 || average_cost < lowest_cost) {
                lowest_cost = average_cost;
            }
            reset();
            return average_cost;
        }
    }

    /** Reset the cost value and the iteration count. */
    void reset() {
        cost = 0;
        iteration = 0;
    }
};


#endif /* COST_HEADER */
