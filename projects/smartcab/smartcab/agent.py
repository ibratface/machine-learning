import random
import operator
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
import pandas as pd

Q_INITIAL = 2
ALPHA = 1

class hashabledict(dict):
    def __hash__(self):
        return hash(frozenset(self.iteritems()))

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env, alpha, q_initial):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.policy = {}
        self.trip_log = []
        self.trip = None
        self.alpha = alpha
        self.q_initial = q_initial

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required
        if self.trip:
            self.trip_log.append(self.trip)
        self.trip = {}
        self.trip['Deadline'] = self.env.get_deadline(self)
        self.trip['Reward'] = 0
        self.trip['Penalty'] = 0

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

        # TODO: Update state
        inputs.pop('right')
        self.state = inputs
        self.state['waypoint'] = self.next_waypoint

        # TODO: Select action according to your policy
        state_hash = hashabledict(frozenset(self.state.iteritems()))
        q = self.policy.get(state_hash, {
            None: self.q_initial,
            'forward': self.q_initial,
            'left': self.q_initial,
            'right': self.q_initial,
        })
        action = max(q.iteritems(), key=operator.itemgetter(1))[0]

        # Execute action and get reward
        reward = self.env.act(self, action)

        # Update trip stats
        self.trip['Reward'] += reward
        self.trip['Remaining'] = self.env.get_deadline(self)
        self.trip['Success'] = self.planner.next_waypoint() == None
        if reward < 0: self.trip['Penalty'] += reward

        # TODO: Learn policy based on state, action, reward
        q[action] = (1 - self.alpha) * q[action] + self.alpha * reward
        self.policy[state_hash] = q

        # print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    record = []
    for q_initial in [0, 2, 10]:
        for alpha in range(1, 6):
            # Set up environment and agent
            e = Environment()  # create environment (also adds some dummy traffic)
            a = e.create_agent(LearningAgent, alpha * 0.2, q_initial)  # create agent
            e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track
            # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

            # Now simulate it
            sim = Simulator(e, update_delay=0, display=False)  # create simulator (uses pygame when display=True, if available)
            # NOTE: To speed up simulation, reduce update_delay and/or set display=False

            sim.run(n_trials=100)  # run for a specified number of trials
            # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line

            a.reset()
            trip_log = pd.DataFrame(a.trip_log)
            # trip_log['Used'] = trip_log['Deadline'] - trip_log['Remaining']
            trip_log['Efficiency'] = trip_log['Remaining'] / trip_log['Deadline'] * 100
            record.append({
                'Success Rate': trip_log[trip_log.Success == True].shape[0],
                'Alpha': alpha * 0.2,
                'Q Initial': q_initial,
                'Efficiency': trip_log['Efficiency'].mean(),
                'Ave Reward': trip_log['Reward'].mean(),
                'Ave Penalty': trip_log['Penalty'].mean(),
            });

    return pd.DataFrame(record)

    # print [ actions for state, actions in a.policy.iteritems() ]
    # policy = pd.DataFrame([ (state, actions) for state, actions in a.policy.iteritems() ])
    # print policy

if __name__ == '__main__':
    print run()
