def grid_coordinates(grid_size):
    coordinates = []
    for i in range(grid_size):
        for j in range(grid_size):
            coordinates.append((i+1, j+1))
    return coordinates

def rectangular_distances(grid_size):
    coordinates = grid_coordinates(grid_size)
    distances = [[0 for _ in range(grid_size ** 2)] for _ in range(grid_size ** 2)]
    for i in range(len(coordinates)):
        for j in range(i+1, len(coordinates)):
            point1 = coordinates[i]
            point2 = coordinates[j]
            distance = abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])
            distances[i][j] = distance
            distances[j][i] = distance
    return distances

def print_matrix(distances):
    for row in distances:
        print(row)

if __name__ == "__main__":
    grid_size = 5
    distances = rectangular_distances(grid_size)
    print_matrix(distances)