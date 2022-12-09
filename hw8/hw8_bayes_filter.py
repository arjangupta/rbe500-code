# This Python script implements the Bayes Filter for the iterations in HW8

# Set verbosity flag
VERBOSE = False

# List of states
all_states = ['open', 'closed']
# List of base beliefs for [open, closed]
# These are the initial beliefs. As we run our 
# iterations, these base beliefs change.
base_belief_list = [0.5, 0.5]

# Helper function for printing base beliefs
def print_base_beliefs():
    print(f'bel(open) = {base_belief_list[0]}')
    print(f'bel(closed) = {base_belief_list[1]}')

# Helper function to get "base" beliefs
def get_base_belief(state) -> float:
    if state == 'open':
        return base_belief_list[0]
    elif state == 'closed':
        return base_belief_list[1]
    else:
        return 0.0

# Helper function for retrieving measurement beliefs
def get_measurement_belief(sense, true_state) -> float:
    if true_state == 'open':
        return 0.6 if sense == 'open' else 0.4
    elif true_state == 'closed':
        return 0.2 if sense == 'open' else 0.8
    else:
        return 0.0

# Helper function for retrieving action beliefs
def get_action_belief(prediction, action, last_known) -> float:
    if action == 'open':
        if last_known == 'open':
            return 1.0 if prediction == 'open' else 0.0
        elif last_known == 'closed':
            return 0.8 if prediction == 'open' else 0.2
        else:
            return 0.0
    elif action == 'do_nothing':
        if last_known == 'open':
            return 1.0 if prediction == 'open' else 0.0
        elif last_known == 'closed':
            return 0.0 if prediction == 'open' else 1.0
        else:
            return 0.0
    else:
        return 0.0

# Declare a data-only class for each filter iteration
class FilterIteration():
    def __init__(self, action: str, measurement: str) -> None:
        self.action =  action
        self.measurement = measurement

# Helper function for the prediction stage of bayes filter
def bf_predict_step(state, action):
    sum = 0
    for s in all_states:
        sum += get_action_belief(prediction=state, action=action, last_known=s)*get_base_belief(state=s)
    return sum

# Helper function for the correction stage of bayes filter
def bf_correct_step(state, measurement, prediction):
    return get_measurement_belief(sense=measurement, true_state=state)*prediction

# Implementation of the Bayes Filter algorithm
def bayes_filter_algorithm(iteration: FilterIteration):
    # Display information about current iteration case
    print(f'The given action is {iteration.action} and given mesurement is {iteration.measurement}')
    # Declare lists for each step
    predictions_list = []
    correction_list = []
    # Loop over [open, closed] to perform prediction and correction steps 
    for s in all_states:
        # Step 1 - prediction stage
        prediction = bf_predict_step(s, iteration.action)
        if VERBOSE:
            print(f'Prediction for {s} is {prediction}')
        predictions_list.append(prediction)
        # Step 2 - correction stage
        correction = bf_correct_step(s, iteration.measurement, prediction=prediction)
        if VERBOSE:
            print(f'Correction for {s} is {correction}')
        correction_list.append(correction)
    # Use correction list to calculate normalizer
    normalizer = 1/sum(correction_list)
    if VERBOSE:
        print(f'The normalizer is {normalizer}')
    # Update beliefs
    global base_belief_list 
    base_belief_list = [b * normalizer for b in correction_list]

def main():
    print("Starting HW8 Bayes Filter")

    filter_iteration_list = [
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open', 'closed'),
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open','open'),
        FilterIteration('do_nothing', 'open')
    ]

    # Print initial beliefs
    print(f'The initial beliefs are:')
    print_base_beliefs()

    # Run the iteration cases
    iteration_number = 1
    for i in filter_iteration_list:
        print('------------------------')
        print(f'Iteration {iteration_number}:')
        # Call the bayes filter for this iteration case
        bayes_filter_algorithm(i)
        # Print the new beliefs
        print(f'The updated beliefs are:')
        print_base_beliefs()
        iteration_number += 1

if __name__ == '__main__':
    main()