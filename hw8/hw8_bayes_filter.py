# This Python script implements the Bayes Filter for the iterations in HW8

# List of states
all_states = ['open', 'closed']

# Helper function to get "base" beliefs
def get_base_belief(state) -> float:
    return 0.5 if state == 'open' or state == 'closed' else 0.0

# Helper function for retrieving measurement beliefs
def get_measurement_belief(sense, measure) -> float:
    if measure == 'open':
        return 0.6 if sense == 'open' else 0.4
    elif measure == 'closed':
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

# Implementation of the Bayes Filter algorithm
def bayes_filter_algorithm(iteration: FilterIteration):
    print(f'The given action is {iteration.action} and mesurement is {iteration.measurement}')
    predictions_list = []
    for s in all_states:
        # Step 1 - prediction stage
        prediction = bf_predict_step(s, iteration.action)
        print(f'Prediction for {s} is {prediction}')
        predictions_list.append(prediction)
        # Step 2 - correction stage

def main():
    print("Starting HW8 Bayes Filter")

    filter_iteration_list = [
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open', 'closed'),
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open','open'),
        FilterIteration('do_nothing', 'open')
    ]

    # Run the iteration cases
    iteration_number = 1
    for i in filter_iteration_list:
        print(f'Iteration {iteration_number}:')
        # Call the bayes filter for this iteration case
        bayes_filter_algorithm(i)
        iteration_number += 1

if __name__ == '__main__':
    main()