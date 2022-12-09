# This Python script implements the Bayes Filter for the iterations in HW8

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