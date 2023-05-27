from gpt4 import *
# Example usage:
probabilities = [0.3,0.4,0.4]
map = generate_random_map(int(2.5/0.1), int(1.5/0.1),probabilities)
print(type(map))
print(map)
#
start = (15, 1)
goal = (1, 1)
#
#
path = astar(map, start, goal)
if path:
    print("Path found:", path)
    display_map(map, path)
else:
    print("Path not found!")