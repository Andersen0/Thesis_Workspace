import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
# Sample data
data = """
4.090426205505783, 0
4.310116610823017, 1
4.048086758146659, 2
4.42021288379448, 3
4.062928933346693, 4
4.3407202324682785, 5
4.114442148407448, 6
4.170714392493489, 7
4.138380474435088, 8
4.130596458381819, 9
4.344114738304303, 10
4.040412274298767, 11
4.322746648236184, 12
4.127460040578046, 13
4.285406375489007, 14
4.185283804850987, 15
4.229258244218438, 16
4.106900752507403, 17
3.8902122658279716, 18
3.679203531022218, 19
3.493682515888121, 20
3.2518675434327124, 21
2.933453774165922, 22
2.747374344656272, 23
2.350269286175539, 24
2.094882223179035, 25
1.9686314424323168, 26
1.528835058854214, 27
1.1138988910382357, 28
0.9471583445594954, 29
0.6271434967774264, 30
0.5678465486704292, 31
0.21879977218732224, 32
0.0, 33
0.0, 34
0.0, 35
0.0, 36
0.0, 37
0.0, 38
0.0, 39
0.0, 40
0.0, 41
0.0, 42
0.0770214857215692, 43
0.2938993579545569, 44
0.6490684161980054, 45
0.8771802258901376, 46
1.0868821630328944, 47
1.309007015538629, 48
1.7050729753971787, 49
2.0540974983159077, 50
2.449961271473606, 51
2.651842876859282, 52
2.825373639455257, 53
3.002491340826897, 54
3.1937044608925818, 55
3.4716978662167546, 56
3.8326950710737933, 57
4.049703530482742, 58
4.2356112087777795, 59
4.105965518183305, 60
4.257938224763071, 61
4.20629431520939, 62
4.155458184817654, 63
4.255712487004132, 64
4.228630269995544, 65
4.1353192964993735, 66
4.273285275240178, 67
4.087815341990362, 68
4.202012839379236, 69
4.161448862361945, 70
4.187905275036941, 71
4.2460700935322535, 72
4.130083117192308, 73
4.170749296837098, 74
4.249548760979912, 75
4.25139333495868, 76
4.153976483813386, 77
4.274413615198481, 78
4.0860856445824165, 79
4.341918390420745, 80
4.122096195392611, 81
4.1534709775031375, 82
4.3126049101121895, 83
4.032368541651903, 84
4.316428790325769, 85
4.123453130454642, 86
4.343318901851184, 87
4.087295691293453, 88
4.179670879134605, 89
4.254667539870925, 90
4.1508666885866905, 91
4.209016488170189, 92
"""
# Parse the data
x_values = []  # Time values
y_values = []  # Speed values

for line in data.strip().split('\n'):
    speed, time = map(float, line.split(', '))  # Splitting each line by comma and space
    x_values.append(time)  # Assign the second value as time
    y_values.append(round(speed, 2))  # Assign the first value as speed, rounded to two decimal places

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values, marker='o', linestyle='-')
plt.xlabel('Time')
plt.ylabel('Speed (km/h)')
plt.grid(True)

# To display a limited number of x-axis labels, we select 10 evenly spaced indices
x_ticks_index = np.linspace(0, len(x_values) - 1, 10, dtype=int)  # Compute indices for 10 labels
x_ticks = [x_values[i] for i in x_ticks_index]  # Extract these indices from x_values
x_labels = [str(x_values[i]) for i in x_ticks_index]  # Extract corresponding y_values for labels, converting to string

plt.xticks(x_ticks, x_labels, rotation=45)  # Set custom ticks and labels on the x-axis

plt.show()