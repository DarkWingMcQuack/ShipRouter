import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

pops_df = pd.read_csv("./qpops.csv")
time_df = pd.read_csv("./time.csv")


def plot(df, titel, ylabel):
    labels = df["#NumberOfNodes"].to_numpy()
    d = df["Dijkstra"].to_numpy()
    ch = df["CH-Dijkstra"].to_numpy()

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width/2, d, width, label='Dijkstra')
    rects2 = ax.bar(x + width/2, ch, width, label='CH-Dijkstra')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel(ylabel)
    ax.set_xlabel("number of nodes in graph")
    ax.set_title(titel)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()

    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)

    fig.tight_layout()

    plt.show()


plot(pops_df, "comparison of heap-pops", "avg heap-pops per query")
plot(time_df, "comparison of runtime", "avg runtime in ms per query")
