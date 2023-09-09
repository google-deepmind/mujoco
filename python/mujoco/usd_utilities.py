
def get_mesh_ranges(nmesh, arr):
    mesh_ranges = [0]
    running_sum = 0
    for i in range(nmesh):
        running_sum += arr[i]
        mesh_ranges.append(running_sum)
    return mesh_ranges