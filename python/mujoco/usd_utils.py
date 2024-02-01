import numpy as np

def create_transform_matrix(rotation_matrix, translation_vector):
  # Ensure rotation_matrix and translation_vector are NumPy arrays
  rotation_matrix = np.array(rotation_matrix)
  translation_vector = np.array(translation_vector)

  transform_matrix = np.eye(4)
  transform_matrix[:3, :3] = rotation_matrix
  transform_matrix[:3, 3] = translation_vector

  return transform_matrix