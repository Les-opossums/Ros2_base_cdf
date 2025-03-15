import numpy as np

list_of_index = [[1, 2, -1],
                 [1, 4, -1],
                 [1, 2, -5],
                 [1, 2, -5],
                 [2, 3, -0], 
                 [1, 2, -1],
                 [1, 2, -1], 
                 ]
list_to_clean = np.array(list_of_index)
list_to_clean = np.unique(list_to_clean, axis=0)
list_to_clean = list_to_clean.tolist()
print(list_to_clean)