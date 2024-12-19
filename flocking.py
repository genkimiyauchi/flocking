import math
from vector2d import Vector2D

# Flocking parameters (repulsion)
TARGET_DISTANCE = 0.1
EXPONENT = 2
GAIN = 1

WEIGHT_ATTRACTION = 10
WEIGHT_REPULSION = 1


def get_attraction_vector(neighbors, my_hop_count):

    result_vector = Vector2D(0,0)

    # If leader is present, move towards the leader
    for robot in neighbors:
        if robot[1]:
            return robot[0]

    # If leader is not present, move towards the neighbors with a smaller hop count to the leader
    num_attract = 0
    for robot in neighbors:
        if robot[1] < my_hop_count:
            result_vector += robot[0]
            num_attract += 1

    if num_attract > 0:
        result_vector /= num_attract

    return result_vector


def repulsion(distance):
    f_norm_dist_exp = pow(TARGET_DISTANCE / distance, EXPONENT)
    return -GAIN / distance * (f_norm_dist_exp * f_norm_dist_exp)


def get_repulsion_vector(neighbors):

    result_vector = Vector2D(0,0)

    for robot in neighbors:
        position = robot[0]
        distance = abs(position)
        angle = math.atan2(position.y, position.x)

        # Calculate repulsion force
        lj_force = repulsion(distance)

        # Calculate the repulsion vector
        x = math.cos(angle) * lj_force
        y = math.sin(angle) * lj_force
        repultion_vector = Vector2D(x, y)

        result_vector += repultion_vector       

    if len(neighbors) > 0:
        result_vector /= len(neighbors)

    return result_vector


def flock(neighbors, my_hop_count):

    attraction_vector = get_attraction_vector(neighbors, my_hop_count)
    repulsion_vector = get_repulsion_vector(neighbors)

    sum_force = WEIGHT_ATTRACTION * attraction_vector + WEIGHT_REPULSION * repulsion_vector

    return sum_force


if __name__ == "__main__":

    # Example usage of the flocking function

    ### Inputs ###

    ### List of neighboring robots
    # Each neighbor has a:
    #   1. position (x,y) relative to the robot
    #   2. the robot's hop count to leader (True if leader)
    neighbors = [
        (Vector2D( 0.1,  0.1), 0), 
        (Vector2D( 0.1,  0.0), 1),
        (Vector2D(-0.1,  0.1), 1),
    ]
    my_hop_count = 1

    ### Output ###

    vector = flock(neighbors, my_hop_count)
    print(vector)

    # Sample output
    # (-2.33333, -0.833333)
    # 
    # The output vector represents the direction to move towards in the time step
    # The magnitude of the vector can be tuned by the parameters defined at the top of the file
