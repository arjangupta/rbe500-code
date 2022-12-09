# This Python script implements the Bayes Filter for the iterations in HW8

# "Base" beliefs
bel_open   = 0.5
bel_closed = 0.5

def get_measurement_belief(sense, measure) -> float:
    if measure == 'open':
        return 0.6 if sense == 'open' else 0.4
    elif measure == 'closed':
        return 0.2 if sense == 'open' else 0.8
    else:
        return 0.0

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


def bayes_algorithm():
    pass

# Declare a data-only class for each filter iteration
class FilterIteration():
    def __init__(self, action: str, measurement: str) -> None:
        self.action =  action
        self.measurement = measurement

def main():
    print("Starting HW8 Bayes Filter")

    filter_iteration_list = [
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open', 'closed'),
        FilterIteration('do_nothing', 'closed'),
        FilterIteration('open','open'),
        FilterIteration('do_nothing', 'open')
    ]

if __name__ == '__main__':
    main()