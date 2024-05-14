import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Constructing the confusion matrices
conf_matrix = np.array([[206, 23],   # TP, FP
                        [0, 183]])    # FN, TN

conf_matrix_wh = np.array([[36, 15],   # TP, FP
                           [0, 17]])   # FN, TN

# Labels for the axes
row_labels = ['True', 'False']
column_labels = ['Worker', 'Adult']

# Creating the first figure for the Worker and Adult Classification
fig1, ax1 = plt.subplots(figsize=(8, 6))
sns.heatmap(conf_matrix, annot=True, fmt='d', cmap='Blues', cbar=False,
            xticklabels=column_labels, yticklabels=row_labels, ax=ax1,
            annot_kws={"size": 16}, linewidths=1, linecolor='black')
ax1.tick_params(axis='both', which='major', labelsize=12)
ax1.tick_params(axis='x', which='both', bottom=False, top=True, labelbottom=False, labeltop=True)
ax1.tick_params(axis='y', which='major', labelsize=12)

# Creating the second figure for the Adult with White Sweater Classification
fig2, ax2 = plt.subplots(figsize=(8, 6))
sns.heatmap(conf_matrix_wh, annot=True, fmt='d', cmap='Blues', cbar=False,
            xticklabels=column_labels, yticklabels=row_labels, ax=ax2,
            annot_kws={"size": 16}, linewidths=1, linecolor='black')
ax2.tick_params(axis='both', which='major', labelsize=12)
ax2.tick_params(axis='x', which='both', bottom=False, top=True, labelbottom=False, labeltop=True)
ax2.tick_params(axis='y', which='major', labelsize=12)

plt.show()
