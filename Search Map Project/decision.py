import numpy as np
import math
from datetime import datetime
from random import randint

from perception import to_polar_coords, pix_to_world

# Base class that all states extend from
class State:
    def run(self):
        assert 0, "run not implemented"
    def update(self):
        assert 0, "update not implemented"
    def next(self):
        assert 0, "next not implemented"

# YouTube URL of this rover in action - https://www.youtube.com/watch?v=FPyKJIwBdjY

class StateMachine:
    def __init__(self, initialState):
        self.current_state = initialState
        self.init_state_once = True

    def run_all(self, Rover):
        if self.init_state_once:
            self.current_state.__init__(Rover)
            self.init_state_once = False
        self.current_state.update(Rover)
        self.current_state.run()
        next_state = self.current_state.next()

        if next_state is self.current_state:
            pass
        else:
            self.current_state = next_state
            self.init_state_once = True

class MappingAndSeeking(State):
    def __init__(self, Rover):
        print("MappingAndSeeking")
        self.rover = Rover
        self.stuck = False
        if self.rover is not None:
            self.rover.throttle = 1

    def update(self, Rover):
        self.rover = Rover

    def run(self):
        if len(self.rover.nav_angles) >= self.rover.stop_forward:
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            if self.rover.vel < self.rover.max_vel:
                # Set throttle value to throttle setting
                self.rover.throttle = self.rover.throttle_set
            else:  # Else coast
                self.rover.throttle = 0
            self.rover.brake = 0
            self.rover.steer = np.clip(refine_nav_angles(self.rover.nav_angles), -15, 15)

            if self.rover.vel == 0 :
                self.stuck = True
                return

        # If there's a lack of navigable terrain pixels then go to 'stop' mode
        elif len(self.rover.nav_angles) < self.rover.stop_forward:
            self.rover.mode = 'stop'
        elif self.rover.vel == 0 :
            self.stuck = True

    def next(self):
        #Transition criteria
        if self.rover.samples_collected == 6 and self.rover.perc_mapped > 95:
            return DecisionsDecisions.gohome
        if self.stuck:
            self.stuck = False
            DecisionsDecisions.return_to_state = self
            return DecisionsDecisions.stuck
        elif self.rover.mode == 'stop':
            return DecisionsDecisions.stop
        elif self.rover.mode == 'goforgold' and not self.rover.picking_up and not self.rover.send_pickup:
            return DecisionsDecisions.pickupgold
        else:
            return self

class Stop(State):
    def __init__(self, Rover):
        print("Stop")
        self.rover = Rover

    def update(self, Rover):
        self.rover = Rover

    def run(self):
        if self.rover.vel > 0.2:
            self.rover.throttle = 0
            self.rover.brake = self.rover.brake_set
            self.rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        elif self.rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            if len(self.rover.nav_angles) < self.rover.go_forward:
                self.rover.throttle = 0
                # Release the brake to allow turning
                self.rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                self.rover.steer = -15  # Could be more clever here about which way to turn

    def next(self):
        if len(self.rover.nav_angles) >= self.rover.go_forward:
            self.rover.mode = 'forward'
            return DecisionsDecisions.mappingandseeking
        else:
            return self

class Stuck(State):
    def __init__(self, Rover):
        print('Stuck')
        self.rover = Rover
        self.random_next_state_int = randint(5, 20)
        self.next_state_integer = 0
        self.rover_steer = -15
        if self.rover is not None and self.rover.nav_angles is not None:
            mean_angles = np.mean(radians_to_degrees(self.rover.nav_angles))
            if mean_angles > 0:
                print('stuck rover_steer - postive')
                self.rover_steer = 15
            else:
                print('stuck rover_steer - negative')
                self.rover_steer = -15

    def update(self, Rover):
        self.rover = Rover

    def run(self):
        self.rover.throttle = 0
        self.rover.brake = 0
        self.rover.steer = self.rover_steer
        self.next_state_integer += 1

    def next(self):
        while self.next_state_integer < self.random_next_state_int:
            return self
        else:
            self.rover.throttle = 1
            return DecisionsDecisions.return_to_state

class PickUpGold(State):
    def __init__(self, Rover):
        print("Get and grab that gold matey")
        self.rover = Rover
        self.first_run = True
        self.go_to_next_state = False
        self.stuck = False
        self.sent_pickup = False

    def update(self, Rover):
        self.rover = Rover

    def run(self):
        if self.rover.picking_up == True and self.sent_pickup == False:
            self.sent_pickup = True
            return
        if self.rover.picking_up == False and self.sent_pickup == True:
            self.go_to_next_state = True
            return

        # If in a state where want to pickup the gold send pickup command
        if self.first_run is True:
            self.first_run = False
            self.rover.brake = self.rover.brake_set
            self.rover.throttle = 0
        else:
            self.rover.brake = 0
            self.rover.throttle = .1
            self.rover.steer = np.clip(np.mean(self.rover.rock_angle * 180 / np.pi), -15, 15)

        if self.rover.vel == 0 \
                and not self.go_to_next_state \
                and not self.rover.near_sample \
                and not self.rover.picking_up \
                and not self.first_run :
            self.stuck = True
            return

        if self.rover.near_sample:
            self.rover.throttle = 0
            self.rover.brake = self.rover.brake_set

        if self.rover.near_sample and self.rover.vel == 0 and not self.rover.picking_up:
            self.rover.send_pickup = True
            return

    def next(self):
        if self.rover.samples_collected == 6 and self.rover.perc_mapped > 95:
            return DecisionsDecisions.gohome
        if self.stuck:
            self.stuck = False
            DecisionsDecisions.return_to_state = self
            return DecisionsDecisions.stuck
        elif self.go_to_next_state and not self.rover.picking_up and not self.rover.send_pickup:
            print("goforgold - go to next state")
            self.rover.mode = 'forward'
            return DecisionsDecisions.mappingandseeking
        else:
            return self

class WhyDontYouGoToYourHome(State):
    def __init__(self, rover):
        print('Why don\'t you go to your home')
        self.rover = rover
        self.stuck = False

    def update(self, Rover):
        self.rover = Rover

    def run(self):
        if len(self.rover.nav_angles) >= self.rover.stop_forward:
            self.rover.brake_set = 0
            self.rover.throttle = self.rover.throttle_set
            origin_bearing, origin_dist = get_bearing_and_dist(self.rover.pos[0], self.rover.pos[1], \
                                                               self.rover.origin_position[0], \
                                                               self.rover.origin_position[1])
            return_angle = np.clip(refine_nav_angles_for_return(self.rover.nav_angles, \
                                                                self.rover.obs_angles, \
                                                                origin_bearing, self.rover.yaw), -15, 15)
            print('return_angle:', return_angle)
            self.rover.steer = return_angle
            if origin_dist < 3:
                print('I\'m home')
                self.rover.throttle = 0
                self.rover.brake = 0
                return
            elif origin_dist < 10:
                print('Super close to home!')
                self.rover.throttle = .05

        if self.rover.vel <= 0.01:
            self.stuck = True
            return

    def next(self):
        if self.stuck:
            self.stuck = False
            DecisionsDecisions.return_to_state = self
            return DecisionsDecisions.stuck
        else:
            return self

#State machine charachater object
class DecisionsDecisions(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, self.mappingandseeking)

#Static state initialization
DecisionsDecisions.stop = Stop(None)
DecisionsDecisions.mappingandseeking = MappingAndSeeking(None)
DecisionsDecisions.pickupgold = PickUpGold(None)
DecisionsDecisions.gohome = WhyDontYouGoToYourHome(None)
DecisionsDecisions.stuck = Stuck(None)
DecisionsDecisions.return_to_state = None
decisions_decisions = DecisionsDecisions()

#Removed the decision tree in favor of a state machine
def decision_step(Rover):
    decisions_decisions.run_all(Rover)
    return Rover

#hug the left wall
def refine_nav_angles(angles):
    mean_direction = np.mean(angles)
    std_deviation = np.std(angles)
    target_angle = mean_direction + (0.45 * std_deviation)
    target_degrees = radians_to_degrees(target_angle)
    return target_degrees

def refine_nav_angles_for_return(angles, obs_angles, bearing, yaw):
    # min_target_angle_degrees = radians_to_degrees(np.min(angles))
    # max_target_angle_degrees = radians_to_degrees(np.max(angles))
    # min_obs_angle_degrees = radians_to_degrees(np.min(obs_angles))
    # max_obs_angle_degrees = radians_to_degrees(np.max(obs_angles))

    bearing_heading_diff = bearing - yaw

    degree_angles = radians_to_degrees(angles)
    mean_angles = np.mean(degree_angles)

    appended_mean_angles = np.append(mean_angles, bearing_heading_diff)
    return np.mean(appended_mean_angles)

def radians_to_degrees(angle):
    return angle * 180/np.pi

def get_bearing_and_dist(startX, startY, endX, endY):
    dLong = endY - startY
    more_math = math.tan(endX / 2.0 + math.pi / 4.0) / math.tan(startX / 2.0 + math.pi / 4.0)
    dPhi = math.log(abs(more_math))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0;
    dist = math.hypot(endX - startX, endY - startY)

    print('bearing: ', bearing, ' distance: ', dist)
    return bearing, dist
