import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Create a confusion matrix / cross table for TP, FP, TN, FN
data = np.array([[32, 17], [0, 15]])  # Example data for confusion matrix
conf_matrix = pd.DataFrame(data, index=["Actual Positive", "Actual Negative"],
                           columns=["Predicted Positive", "Predicted Negative"])

# Create a heatmap for the confusion matrix
plt.figure(figsize=(8, 6))
heatmap = sns.heatmap(conf_matrix, annot=True, fmt="d", cmap="Blues", cbar=False,
                      annot_kws={"size": 16}, linewidths=1, linecolor='black')

# Set the labels for axes (removing xlabel and ylabel for cleaner look)
heatmap.set_xlabel(None)
heatmap.set_ylabel(None)
heatmap.set_xticklabels(["Worker", "Adult"])
heatmap.set_yticklabels(["Predicted Worker", "Predicted Adult"])

# Move x-axis labels to the top and adjust label alignment for clarity
heatmap.xaxis.tick_top()
plt.xticks(rotation=0, ha='center')
plt.yticks(rotation=0, va='center')

plt.show()